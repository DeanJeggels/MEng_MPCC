// mpcc_solver_benchmark.cpp
// Standalone benchmarking node that measures MPCC solver performance
// Uses recorded path from path_recorder.cpp and replicates MPCC controller setup

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>
#include <mpc/NLMPC.hpp>
#include <mpc/Utils.hpp>
#include <mpc/IMPC.hpp>
#include <mpc/Logger.hpp>
#include <chrono>

// Same constants as your MPCC controller
constexpr int Tnx = 4; // [x, y, psi, s]
constexpr int Tnu = 3; // [v, ω, v_s]
constexpr int Tny = 4;
constexpr int Tph = 3;
constexpr int Tch = 3;
constexpr int iterations = 1000;
constexpr int TRACK_BD_CONSTR = 0*(Tph+1);
constexpr int MAX_OBSTACLES = 0;
constexpr int OBSTACLE_CONSTR = MAX_OBSTACLES*(Tph+1);
constexpr int Tineq = 6*(Tph+1) + TRACK_BD_CONSTR + OBSTACLE_CONSTR;
constexpr int Teq = 0;
constexpr double PI_2 = M_PI / 2.0;

mpc::NLMPC<Tnx,Tnu,Tny,Tph,Tch,Tineq,Teq> controller_;

// Copy all your structures from the original MPCC controller
struct TrajectoryPoint {
    double x, y;
    double left_bound, right_bound;
};

struct PathDerivatives {
    double dx_ds, dy_ds;
    double theta;
};

struct Obstacle {
    double s, vs, x, y, radius, safety_dist;
    
    std::pair<double, double> getCurrentPosition(const class LinearSpline& spline) const;
    std::pair<double, double> predictPosition(double dt, int steps, const class LinearSpline& spline) const;
    void update(double dt, double track_length = 0.0);
};

struct NavigationState {
    double x, y, psi, s;
    double v, omega, v_s;
};

struct SolverResult {
    double solve_time_ms;
    bool converged;
    int iterations_used;
    double cost_value;
    int step_number;
};

// Copy your complete LinearSpline class from mpcc_controller.cpp
class LinearSpline {
private:
    std::vector<TrajectoryPoint> trajectory_;
    std::vector<double> s_;
    std::vector<PathDerivatives> derivatives_;
    double total_length_ = 0.0;
    mutable size_t last_index_ = 0;

public:
    bool loadFromCSV(const std::string &file) {
        std::ifstream in(file);
        if (!in) return false;
        
        trajectory_.clear();
        s_.clear();
        derivatives_.clear();
        
        std::string line;
        std::getline(in, line); // skip header
        
        while (std::getline(in, line)) {
            std::stringstream ss(line);
            TrajectoryPoint pt;
            
            if (!(ss >> pt.x)) break;
            if (ss.peek() == ',') ss.ignore();
            if (!(ss >> pt.y)) break;
            if (ss.peek() == ',') ss.ignore();
            if (!(ss >> pt.right_bound)) break;
            if (ss.peek() == ',') ss.ignore();
            if (!(ss >> pt.left_bound)) break;
            
            trajectory_.push_back(pt);
        }
        
        if (trajectory_.empty()) return false;
        
        // Compute arc lengths and derivatives (same as original)
        s_.resize(trajectory_.size());
        s_[0] = 0;
        for (size_t i = 1; i < trajectory_.size(); ++i) {
            const auto &a = trajectory_[i-1], &b = trajectory_[i];
            double dx = b.x - a.x, dy = b.y - a.y;
            s_[i] = s_[i-1] + std::sqrt(dx*dx + dy*dy);
        }
        total_length_ = s_.back();
        
        derivatives_.resize(trajectory_.size());
        for (size_t i = 0; i < trajectory_.size(); ++i) {
            size_t next_i = std::min(i + 1, trajectory_.size() - 1);
            size_t prev_i = (i > 0) ? i - 1 : i;
            double ds = s_[next_i] - s_[prev_i];
            
            if (ds < 1e-6) {
                derivatives_[i] = {1.0, 0.0, 0.0};
            } else {
                const auto &A = trajectory_[prev_i], &B = trajectory_[next_i];
                derivatives_[i].dx_ds = (B.x - A.x) / ds;
                derivatives_[i].dy_ds = (B.y - A.y) / ds;
                derivatives_[i].theta = std::atan2(derivatives_[i].dy_ds, derivatives_[i].dx_ds);
            }
        }
        return true;
    }
    
    TrajectoryPoint getPoint(double s) const {
        if (trajectory_.empty()) return {0.0, 0.0, 0.0, 0.0};
        if (s <= s_.front()) return trajectory_.front();
        if (s >= s_.back()) return trajectory_.back();
        
        size_t i = last_index_;
        if (i >= s_.size() - 1 || s < s_[i] || s > s_[i+1]) {
            auto it = std::lower_bound(s_.begin(), s_.end(), s);
            i = std::max(size_t(0), (size_t)(it - s_.begin()) - 1);
            last_index_ = i;
        }
        
        double t = (s - s_[i]) / (s_[i+1] - s_[i]);
        const auto &A = trajectory_[i], &B = trajectory_[i+1];
        return {
            A.x + t*(B.x - A.x), A.y + t*(B.y - A.y),
            A.left_bound + t*(B.left_bound - A.left_bound),
            A.right_bound + t*(B.right_bound - A.right_bound)
        };
    }
    
    const PathDerivatives& getDerivatives(double s) const {
        if (derivatives_.empty()) {
            static const PathDerivatives default_deriv = {1.0, 0.0, 0.0};
            return default_deriv;
        }
        if (s <= s_.front()) return derivatives_.front();
        if (s >= s_.back()) return derivatives_.back();
        
        auto it = std::lower_bound(s_.begin(), s_.end(), s);
        size_t i = std::max(size_t(0), (size_t)(it - s_.begin()) - 1);
        return derivatives_[i];
    }
    
    std::pair<double, double> getBoundaries(double s) const {
        auto point = getPoint(s);
        return {point.left_bound, point.right_bound};
    }
    
    double getTotalLength() const { return total_length_; }
};

class MPCCSolverBenchmark : public rclcpp::Node {
private:
    // Copy your exact Constants structure
    struct Constants {
        const double v_max, ω_max, v_s_max, rate, dt;
        const double vehicle_radius = 0.6;
        const double safety_distance = 0.8;
        const double k_v = 0.1, k_ω = 1.0, k_vs = 0.1;
        const double k_c = 100.0, k_l = 100.0, k_th = 10000.0;
        const double Pk = 300.0;
        const double qθ = k_th / 100.0;
        const Eigen::Matrix2d Q;
        const Eigen::Matrix3d R;
        
        Constants(double v_max, double ω_max, double v_s_max, double rate)
            : v_max(v_max), ω_max(ω_max), v_s_max(v_s_max), rate(rate), dt(1.0/rate),
              Q((Eigen::Matrix2d() << k_c/(0.5*0.5), 0, 0, k_l/(0.5*0.5)).finished()),
              R((Eigen::Matrix3d() << k_v/(v_max*v_max), 0, 0, 0, k_ω/(ω_max*ω_max), 0, 0, 0, k_vs/(v_s_max*v_s_max)).finished()) {}
    };

    Constants constants_;
    LinearSpline spline_;
    std::vector<NavigationState> navigation_states_;
    std::vector<SolverResult> benchmark_results_;
    std::vector<Obstacle> obstacles_;
    
public:
    MPCCSolverBenchmark() : Node("mpcc_solver_benchmark"), constants_(0.2, PI_2, 0.2, 5.0) {
        // Load reference spline (same path as your controller)
        if (!spline_.loadFromCSV("/home/deanjeggs/new_ros2_ws/src/mpcc/racetracks/aut_centerline copy.csv")) {
            RCLCPP_ERROR(get_logger(), "Failed to load reference spline");
            return;
        }
        
        setupObstacles();
        setupController();
        
        if (!generateNavigationStates()) {
            RCLCPP_ERROR(get_logger(), "Failed to generate navigation states");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "Generated %zu navigation states", navigation_states_.size());
        runBenchmark();
    }

private:
    void setupObstacles() {
        obstacles_.clear();
        // Obstacle obs1;
        // obs1.s = 7.0; obs1.vs = 0.03; obs1.x = 2.05; obs1.y = 0.0;
        // obs1.radius = 0.6; obs1.safety_dist = constants_.safety_distance;
        // obstacles_.push_back(obs1);
    }
    
    void setupController() {
        controller_.setLoggerLevel(mpc::Logger::LogLevel::NORMAL);

        // Capture constants by reference to avoid copies
        const auto& consts = constants_;
        const auto& obs_ref = obstacles_;

        controller_.setIneqConFunction(
        [this, &consts, &obs_ref](mpc::cvec<Tineq>& ineq, const mpc::mat<Tph + 1, Tnx>& x,  
         const mpc::mat<Tph + 1, Tny>&,  const mpc::mat<Tph + 1, Tnu>& u, const double&) {
            int idx = 0;

            for (int i = 0; i <= Tph; ++i) {
                // Input constraints
                ineq(idx++) = u(i,0) - consts.v_max;
                ineq(idx++) = -u(i,0) + 0.2*consts.v_max;
                ineq(idx++) = u(i,1) - consts.ω_max;
                ineq(idx++) = -u(i,1) - consts.ω_max;
                ineq(idx++) = u(i,2) - consts.v_s_max;
                ineq(idx++) = -u(i,2) + 0.2*consts.v_s_max;

                // Track boundary constraints
                double s = x(i,3);
                auto ref = spline_.getPoint(s);
                const auto& deriv = spline_.getDerivatives(s);
                double dx = x(i,0) - ref.x;
                double dy = x(i,1) - ref.y;
                double e_l = -std::cos(deriv.theta)*dx - std::sin(deriv.theta)*dy;
            }
        }, 1e-6);

        controller_.setStateSpaceFunction(
            [&consts](mpc::cvec<Tnx>& x_next, const mpc::cvec<Tnx>& x, const mpc::cvec<Tnu>& u, const unsigned int&){
                double cos_psi = std::cos(x(2));
                double sin_psi = std::sin(x(2));
                x_next(0) = x(0) + u(0)*cos_psi*consts.dt;
                x_next(1) = x(1) + u(0)*sin_psi*consts.dt;
                x_next(2) = x(2) + u(1)*consts.dt;
                x_next(3) = x(3) + u(2)*consts.dt;
            });

        controller_.setOutputFunction(
            [](auto &y, const auto &x, const auto&, unsigned){ y = x; });

        controller_.setObjectiveFunction(
            [this, &consts, &obs_ref](const mpc::mat<Tph + 1, Tnx>& X, const mpc::mat<Tph + 1, Tny>&, 
             const mpc::mat<Tph + 1, Tnu>& U, const double&){
                double cost = 0.0;
                Eigen::Vector3d prev_u = Eigen::Vector3d::Zero();

                for (int i = 0; i <= Tph; ++i) {
                    double s = X(i,3);
                    auto ref = spline_.getPoint(s);
                    const auto& d = spline_.getDerivatives(s);
                    double dx = X(i,0) - ref.x;
                    double dy = X(i,1) - ref.y;
                    double e_c = std::sin(d.theta)*dx - std::cos(d.theta)*dy;
                    double e_l = -std::cos(d.theta)*dx - std::sin(d.theta)*dy;

                    // Use pre-computed Q matrix
                    cost += consts.Q(0,0)*e_c*e_c + consts.Q(1,1)*e_l*e_l;
                    cost -= consts.qθ*s;

                    // Control effort
                    Eigen::Vector3d u_curr(U(i,0), U(i,1), U(i,2));
                    Eigen::Vector3d du = u_curr - prev_u;
                    cost += du.transpose() * consts.R * du;
                    prev_u = u_curr;
                }
                return cost;
            });

        mpc::NLParameters p;
        p.maximum_iteration = iterations;
        p.relative_ftol = 1e-6;
        p.relative_xtol = 1e-6;
        p.absolute_ftol = 1e-6;
        p.absolute_xtol = 1e-6;
        p.hard_constraints = false;
        p.enable_warm_start = true;
        controller_.setOptimizerParameters(p);
    }
    
    bool generateNavigationStates() {
        // Load recorded path from your path_recorder output
        std::ifstream file("recorded_path.csv");
        if (!file.is_open()) return false;
        
        std::string line;
        std::getline(file, line); // Skip header
        
        std::vector<std::pair<double, double>> path_points;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            double x, y;
            
            if (std::getline(ss, cell, ',')) x = std::stod(cell);
            if (std::getline(ss, cell, ',')) y = std::stod(cell);
            path_points.emplace_back(x, y);
        }
        
        // Generate navigation states from recorded path
        navigation_states_.clear();
        double current_s = 0.0;
        Eigen::Vector3d current_u(0.0, 0.0, 0.2*constants_.v_s_max);
        
        for (size_t i = 0; i < path_points.size(); ++i) {
            NavigationState state;
            state.x = path_points[i].first;
            state.y = path_points[i].second;
            
            if (i < path_points.size() - 1) {
                double dx = path_points[i+1].first - path_points[i].first;
                double dy = path_points[i+1].second - path_points[i].second;
                state.psi = std::atan2(dy, dx);
                current_s += std::sqrt(dx*dx + dy*dy);
            } else {
                state.psi = (i > 0) ? std::atan2(path_points[i].first - path_points[i-1].first,
                                               path_points[i].second - path_points[i-1].second) : 0.0;
            }
            
            state.s = current_s;
            if (state.s >= spline_.getTotalLength()) {
                state.s -= spline_.getTotalLength();
                current_s = state.s;
            }
            
            state.v = current_u(0); state.omega = current_u(1); state.v_s = current_u(2);
            navigation_states_.push_back(state);
            
            // Update control for next iteration
            current_u(0) = std::min(0.15, current_u(0) + 0.01);
            current_u(1) *= 0.9;
            current_u(2) = 0.15;
        }
        
        return !navigation_states_.empty();
    }
    
    void runBenchmark() {
        RCLCPP_INFO(get_logger(), "Starting benchmark with %zu states", navigation_states_.size());
        
        Eigen::Vector<double, Tnu> u0;
        u0 << 0.0, 0.0, 0.2*constants_.v_s_max;
        
        for (size_t step = 0; step < navigation_states_.size(); ++step) {
            const auto& nav_state = navigation_states_[step];
            
            Eigen::Vector<double, Tnx> x0;
            x0 << nav_state.x, nav_state.y, nav_state.psi, nav_state.s;
            u0 << nav_state.v, nav_state.omega, nav_state.v_s;
            
            // CRITICAL: Measure solver time
            auto start_time = std::chrono::high_resolution_clock::now();
            auto result = controller_.optimize(x0, u0);
            auto end_time = std::chrono::high_resolution_clock::now();
            
            double solve_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            
            SolverResult bench_result;
            bench_result.solve_time_ms = solve_time_ms;

            bench_result.step_number = static_cast<int>(step);
            benchmark_results_.push_back(bench_result);
            
            if ((step + 1) % 20 == 0) {
                RCLCPP_INFO(get_logger(), "Progress: %zu/%zu - Last solve: %.3f ms", 
                           step + 1, navigation_states_.size(), solve_time_ms);
            }
        }
        
        saveBenchmarkResults();
        generateStatistics();
        rclcpp::shutdown();
    }
    
    void saveBenchmarkResults() {
        std::ofstream file("solver_benchmark_results.csv");
        file << "step,solve_time_ms,converged,iterations_used,cost_value\n";
        
        for (const auto& result : benchmark_results_) {
            file << result.step_number << "," << result.solve_time_ms << ","
                 << (result.converged ? 1 : 0) << "," << result.iterations_used << ","
                 << result.cost_value << "\n";
        }
        file.close();
        RCLCPP_INFO(get_logger(), "Saved results to: solver_benchmark_results.csv");
    }
    
    void generateStatistics() {
        std::vector<double> solve_times;
        int converged_count = 0;
        
        for (const auto& result : benchmark_results_) {
            solve_times.push_back(result.solve_time_ms);
            if (result.converged) converged_count++;
        }
        
        std::sort(solve_times.begin(), solve_times.end());
        
        double mean = std::accumulate(solve_times.begin(), solve_times.end(), 0.0) / solve_times.size();
        double median = solve_times[solve_times.size() / 2];
        
        RCLCPP_INFO(get_logger(), "\n========== BENCHMARK RESULTS ==========");
        RCLCPP_INFO(get_logger(), "Total calls: %zu", benchmark_results_.size());
        RCLCPP_INFO(get_logger(), "Convergence: %.2f%%", (double)converged_count/benchmark_results_.size()*100);
        RCLCPP_INFO(get_logger(), "Mean solve time: %.4f ms", mean);
        RCLCPP_INFO(get_logger(), "Median solve time: %.4f ms", median);
        RCLCPP_INFO(get_logger(), "Min: %.4f ms, Max: %.4f ms", solve_times.front(), solve_times.back());
        RCLCPP_INFO(get_logger(), "========================================");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCCSolverBenchmark>());
    rclcpp::shutdown();
    return 0;
}
