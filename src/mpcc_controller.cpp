// mpcc_controller_optimized.cpp

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

constexpr int Tnx = 4;   // [x, y, psi, s]
constexpr int Tnu = 3;   // [v, ω, v_s]
constexpr int Tny = 4;
constexpr int Tph = 3;
constexpr int Tch = 3;
constexpr int iterations = 1000;

constexpr int TRACK_BD_CONSTR = 0*(Tph+1);
constexpr int MAX_OBSTACLES =1;
constexpr int OBSTACLE_CONSTR = MAX_OBSTACLES*(Tph+1);
constexpr int Tineq = 6*(Tph+1) + TRACK_BD_CONSTR + OBSTACLE_CONSTR;
constexpr int Teq = 0;

// Pre-allocate commonly used constants
constexpr double PI_2 = M_PI / 2.0;

mpc::NLMPC<Tnx,Tnu,Tny,Tph,Tch,Tineq,Teq> controller_;

struct TrajectoryPoint { 
    double x, y; 
    double left_bound, right_bound; 
};

struct PathDerivatives { 
    double dx_ds, dy_ds;
    double theta; // Pre-computed theta
};

struct Obstacle {
    double s;
    double vs;
    double x, y;       
    double radius;     
    double safety_dist;

    std::pair<double, double> getCurrentPosition(const class LinearSpline& spline) const;
    std::pair<double, double> predictPosition(double dt, int steps, const class LinearSpline& spline) const;
    void update(double dt, double track_length = 0.0);
};

class LinearSpline {
private:
    std::vector<TrajectoryPoint> trajectory_;
    std::vector<double> s_;
    std::vector<PathDerivatives> derivatives_; // Pre-computed derivatives
    double total_length_ = 0.0;

    // Cache for last lookup (spatial locality optimization)
    mutable size_t last_index_ = 0;

public:

            // Add this to the LinearSpline class (in the public section):
        double getSAtIndex(size_t idx) const {
            if (idx >= s_.size()) return s_.back();
            return s_[idx];
        }

    bool loadFromCSV(const std::string &file) {
        std::ifstream in(file);
        if (!in) return false;

        trajectory_.clear();
        s_.clear();
        derivatives_.clear();

        trajectory_.reserve(1000); // Pre-allocate reasonable size
        s_.reserve(1000);
        derivatives_.reserve(1000);

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

        // Compute arc lengths
        s_.resize(trajectory_.size());
        s_[0] = 0;
        for (size_t i = 1; i < trajectory_.size(); ++i) {
            const auto &a = trajectory_[i-1], &b = trajectory_[i];
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double ds = std::sqrt(dx*dx + dy*dy); // Faster than hypot for this case
            s_[i] = s_[i-1] + ds;
        }

        total_length_ = s_.back();

        // Pre-compute derivatives and theta values
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

    // Optimized point lookup with caching
    TrajectoryPoint getPoint(double s) const {
        if (trajectory_.empty()) return {0.0, 0.0, 0.0, 0.0};
        if (s <= s_.front()) return trajectory_.front();
        if (s >= s_.back()) return trajectory_.back();

        // Try cached index first (spatial locality)
        size_t i = last_index_;
        if (i >= s_.size() - 1 || s < s_[i] || s > s_[i+1]) {
            // Cache miss, do binary search
            auto it = std::lower_bound(s_.begin(), s_.end(), s);
            i = std::max<size_t>(0, (it - s_.begin()) - 1);
            last_index_ = i; // Update cache
        }

        double t = (s - s_[i]) / (s_[i+1] - s_[i]);
        const auto &A = trajectory_[i], &B = trajectory_[i+1];

        return { 
            A.x + t*(B.x - A.x), 
            A.y + t*(B.y - A.y),
            A.left_bound + t*(B.left_bound - A.left_bound),
            A.right_bound + t*(B.right_bound - A.right_bound)
        };
    }

    // Use pre-computed derivatives
    const PathDerivatives& getDerivatives(double s) const {
        if (derivatives_.empty()) {
            static const PathDerivatives default_deriv = {1.0, 0.0, 0.0};
            return default_deriv;
        }

        if (s <= s_.front()) return derivatives_.front();
        if (s >= s_.back()) return derivatives_.back();

        auto it = std::lower_bound(s_.begin(), s_.end(), s);
        size_t i = std::max<size_t>(0, (it - s_.begin()) - 1);
        return derivatives_[i];
    }

    std::pair<double, double> getBoundaries(double s) const {
        if (trajectory_.empty()) return {0,0};
        if (s <= s_.front()) return {trajectory_.front().left_bound, trajectory_.front().right_bound};
        if (s >= s_.back())  return {trajectory_.back().left_bound, trajectory_.back().right_bound};

        auto it = std::lower_bound(s_.begin(), s_.end(), s);
        size_t i = std::max<size_t>(0, (it - s_.begin()) - 1);
        double t = (s - s_[i]) / (s_[i+1] - s_[i]);
        const auto &A = trajectory_[i], &B = trajectory_[i+1];

        double left = A.left_bound + t*(B.left_bound - A.left_bound);
        double right = A.right_bound + t*(B.right_bound - A.right_bound);
        return {left, right};
    }

    const std::vector<TrajectoryPoint>& points() const { return trajectory_; }
    double getTotalLength() const { return total_length_; }
};

// Obstacle method implementations
std::pair<double, double> Obstacle::getCurrentPosition(const LinearSpline& spline) const {
    auto current_point = spline.getPoint(s);
    return {current_point.x, current_point.y + 0.45};  
}

std::pair<double, double> Obstacle::predictPosition(double dt, int steps, const LinearSpline& spline) const {
    double future_s = s + vs * dt * steps;
    auto future_point = spline.getPoint(future_s);
    return {future_point.x, future_point.y + 0.45};  
}

void Obstacle::update(double dt, double track_length) {
    s += vs * dt;

    if (track_length > 0.0 && s >= track_length) {
        s -= track_length;
    }
}

class MPCCController : public rclcpp::Node {
private:
    // Constants grouped together for better cache locality
    struct Constants {
        const double v_max, ω_max, v_s_max, rate, dt;
        const double vehicle_radius = 0.6;
        const double safety_distance = 0.8;
        const double k_v = 0.1, k_ω = 1.0, k_vs = 0.1;
        const double k_c = 50, k_l = 50, k_th = 10000.0;
        const double Pk = 100.0;
        const double qθ = k_th / 100.0;
        const Eigen::Matrix2d Q;
        const Eigen::Matrix3d R;

        Constants(double v_max, double ω_max, double v_s_max, double rate)
        : v_max(v_max), ω_max(ω_max), v_s_max(v_s_max), rate(rate), dt(1.0/rate),
          Q((Eigen::Matrix2d() << k_c, 0, 0, k_l).finished()),
          R((Eigen::Matrix3d() << 
             k_v/(v_max*v_max), 0, 0,
             0, k_ω/(ω_max*ω_max), 0,
             0, 0, k_vs/(v_s_max*v_s_max)).finished()) {}
    };

   struct ControllerState {
        Eigen::Vector<double, Tnx> x0;
        Eigen::Vector<double, Tnu> u0;
        double track_length = 0.0;
        std::vector<double> lap_times;

        int laps_completed = 0;
        double lap_start_time = 0.0;
        
        // Error accumulators
        double contour_error_sum = 0.0;
        double lag_error_sum = 0.0;
        int error_sample_count = 0;
        double solver_time_sum = 0.0;
        int solver_call_count = 0;
        
        // Line crossing detection
        double prev_x = 0.0;
        double prev_y = 0.0;
        bool first_position_set = false;
    } state_;



    // Configuration
    Constants constants_;
    int desired_laps_ = 10;
    bool multi_lap_mode_ = true;

    // Overtake recording variables
    bool recording_overtake = false;
    std::vector<double> current_dv2o;
    std::vector<double> current_s;

    struct OvertakeSnippet {
        std::vector<double> dv2o_snippet;
        std::vector<double> s_snippet;
        double min_dv2o;
        double s_at_min;
    };
    std::vector<OvertakeSnippet> overtake_snippets;

   


    std::vector<double> contour_errors_;
    std::vector<double> lag_errors_;
    std::vector<double> avg_solver_times_; // Average solver time per lap
    std::string robot_name_ = "mpcc";
    // Data structures
    LinearSpline spline_;
    std::vector<Obstacle> obstacles_;
    std::vector<geometry_msgs::msg::PoseStamped> actual_path_;

    // Publishers and Subscribers - grouped for better memory layout
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_, actual_pub_, predicted_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr states_pub_, inputs_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_, opt_time_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr reference_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundary_pub_, obstacle_pub_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr obstacle_actual_sub_, obstacle_predicted_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_current_sub_;

public:
    MPCCController()
    : Node("mpcc_controller"),
      constants_(0.2, PI_2, 0.2, 5.0)
    {
        // Pre-allocate vectors
        contour_errors_.reserve(desired_laps_);
            lag_errors_.reserve(desired_laps_);
                avg_solver_times_.reserve(desired_laps_); // Add this line
            actual_path_.reserve(10000);
    
        // if (!spline_.loadFromCSV("/home/deanjeggs/new_ros2_ws/src/mpcc/racetracks/aut_centerline copy.csv")) {
        // if (!spline_.loadFromCSV("/home/deanjeggs/new_ros2_ws/src/mpcc/racetracks/recorded_path_straight.csv")) {
        if (!spline_.loadFromCSV("/home/deanjeggs/new_ros2_ws/src/mpcc/racetracks/figure8.csv")) {
            RCLCPP_ERROR(get_logger(), "Failed loading spline");
            rclcpp::shutdown();
            return;
        }

        state_.track_length = spline_.getTotalLength();
        state_.lap_start_time = this->now().seconds();

        RCLCPP_INFO_ONCE(get_logger(), "Track length: %.2f meters, Target laps: %d", 
                        state_.track_length, desired_laps_);

        setupSubscribers();
        setupPublishers();
        publishReference();
        // publishTrackBoundaries();
        setupController();
        setupObstacles();
        initState();
    }

private:

    std::pair<double, double> calculateErrors() const {
            auto ref = spline_.getPoint(state_.x0(3));
            const auto& deriv = spline_.getDerivatives(state_.x0(3));
            
            double dx = state_.x0(0) - ref.x;
            double dy = state_.x0(1) - ref.y;
            
            // Contour error (lateral deviation)
            double e_c = std::sin(deriv.theta) * dx - std::cos(deriv.theta) * dy;
            
            // Lag error (longitudinal deviation)
            double e_l = -std::cos(deriv.theta) * dx - std::sin(deriv.theta) * dy;
            
            return {std::abs(e_c), std::abs(e_l)};
        }

    void setupSubscribers() {
        using std::placeholders::_1;
        // model_states_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
        //     "/model_states", 10,
        //     std::bind(&MPCCController::odomCallback, this, _1));

                    obstacle_actual_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/mpcc_obst/actual_path", 10,
            std::bind(&MPCCController::obstacleActualPathCallback, this, _1));

        obstacle_predicted_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/mpcc_obst/predicted_path", 10,
            std::bind(&MPCCController::obstaclePredictedPathCallback, this, _1));

        obstacle_current_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/mpcc_obst/current_reference_point", 10,
            std::bind(&MPCCController::obstacleCurrentPointCallback, this, _1));

             localization_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization_pose", 10,
            std::bind(&MPCCController::localizationPoseCallback, this, std::placeholders::_1));
    }

    void setupPublishers() {
        reference_pub_ = create_publisher<nav_msgs::msg::Path>("/mpcc/reference_path", 10);
        predicted_pub_ = create_publisher<nav_msgs::msg::Path>("/mpcc/predicted_path", 10);
        states_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("mpcc/optimal_states", 10);
        inputs_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("mpcc/optimal_inputs", 10);
        cmd_pub_ = create_publisher<nav_msgs::msg::Odometry>("/mpcc/result_cmd", 10);
        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        error_pub_ = create_publisher<std_msgs::msg::Float64>("mpcc/tracking_error", 10);
        opt_time_pub_ = create_publisher<std_msgs::msg::Float64>("mpcc/opt_time", 10);
        actual_pub_ = create_publisher<nav_msgs::msg::Path>("/mpcc/actual_path", 10);
        reference_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/mpcc/current_reference_point", 10);
        obstacle_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mpcc/obstacles", 10);
        boundary_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mpcc/boundaries", 10);

    }

    // Inline frequently called functions
    inline double fastHypot(double x, double y) const {
        return std::sqrt(x*x + y*y);
    }

    void obstacleActualPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {

    }

    void obstaclePredictedPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    
    }

    void obstacleCurrentPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    
    }

    inline double computeV2ODistance(double vehicle_x, double vehicle_y,
                                   double obstacle_x, double obstacle_y,
                                   double obstacle_radius) const {
        double dx = vehicle_x - obstacle_x;
        double dy = vehicle_y - obstacle_y;
        return fastHypot(dx, dy) - obstacle_radius - constants_.vehicle_radius;
    }

    inline double getAdaptiveObstacleWeight(double DV2O, double DSft_O) const {
        if (DV2O < 0) {
            return constants_.Pk;
        } else if (DV2O <= DSft_O && DV2O >= 0) {
            double ratio = DV2O / DSft_O;
            return constants_.Pk * std::exp(-2.0 * ratio * ratio);
        }
        return 0.0;
    }

    void setupObstacles() {
        obstacles_.clear();
        Obstacle obs1;
        obs1.s = 10.0;
        obs1.vs = 0.03;
        obs1.x = 2.05;
        obs1.y = 0.0;
        obs1.radius = 0.6;
        obs1.safety_dist = constants_.safety_distance; 
        obstacles_.push_back(obs1);

        RCLCPP_INFO(get_logger(), "Added %zu obstacles", obstacles_.size());
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

                auto [left_bd, right_bd] = spline_.getBoundaries(s);
                // ineq(idx++) = e_l - (left_bd - consts.vehicle_radius);
                // ineq(idx++) = -e_l - (right_bd - consts.vehicle_radius);

                // Obstacle constraints
                double x_veh = x(i,0);
                double y_veh = x(i,1);
                double pred_time = consts.dt * i;  

                for (const auto& obstacle : obs_ref) {
                    double future_s = obstacle.s + obstacle.vs * pred_time;
                    auto obs_point = spline_.getPoint(future_s);
                    double obs_x = obs_point.x;
                    double obs_y = obs_point.y + 0.1;

                    double dx_obs = x_veh - obs_x;
                    double dy_obs = y_veh - obs_y;
                    double D_v2o = std::sqrt(dx_obs*dx_obs + dy_obs*dy_obs) - obstacle.radius - consts.vehicle_radius;

                    ineq(idx++) = -D_v2o + obstacle.safety_dist;
                }

                // Fill remaining obstacle slots if using fixed constraint size
                for (size_t j = obs_ref.size(); j < MAX_OBSTACLES; ++j) {
                    ineq(idx++) = -1000.0;
                }
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

                    // // Obstacle avoidance
                    // for (const auto& obstacle : obs_ref) {
                    //     std::pair<double, double> obs_pos;

                    //     if (std::abs(obstacle.vs) < 1e-6) {
                    //         obs_pos = obstacle.getCurrentPosition(spline_);
                    //     } else {
                    //         obs_pos = obstacle.predictPosition(consts.dt, i, spline_);
                    //     }

                    //     double DV2O = computeV2ODistance(X(i,0), X(i,1), 
                    //                                     obs_pos.first, obs_pos.second,
                    //                                     obstacle.radius);
                    //     double qV2O = getAdaptiveObstacleWeight(DV2O, obstacle.safety_dist);

                    //     if (qV2O > 0) {
                    //         double eV2O = DV2O - obstacle.safety_dist;
                    //         cost += qV2O * eV2O * eV2O; 
                    //     }
                    // }

                    // Control effort
                    Eigen::Vector3d u_curr(U(i,0), U(i,1), U(i,2));
                    Eigen::Vector3d du = u_curr - prev_u;
                    cost += du.transpose() * consts.R * du;
                    prev_u = u_curr;
                }
                return cost;
            });

        // controller_.setIneqConFunction([this, &consts, &obs_ref](
        //     mpc::cvec<Tineq>& ineq,
        //     const mpc::mat<Tph+1, Tnx>& X,
        //     const mpc::mat<Tph+1, Tny>&,
        //     const mpc::mat<Tph+1, Tnu>& U,
        //     const double&) {
            
        //     int idx = 0;
        //     const int horizon = Tph + 1;  // 4 steps (0 to Tph)
            
        //     // Extract control input columns for vectorized operations
        //     Eigen::Map<const Eigen::VectorXd> u_v(U.col(0).data(), horizon);
        //     Eigen::Map<const Eigen::VectorXd> u_w(U.col(1).data(), horizon);
        //     Eigen::Map<const Eigen::VectorXd> u_vs(U.col(2).data(), horizon);
            
        //     // ===== VECTORIZED INPUT CONSTRAINTS (6 per timestep) =====
        //     // Upper bounds
        //     ineq.segment(idx, horizon) = u_v.array() - consts.v_max;
        //     idx += horizon;
            
        //     // Lower bounds
        //     ineq.segment(idx, horizon) = -u_v.array() + 0.2 * consts.v_max;
        //     idx += horizon;
            
        //     // Angular velocity upper bounds
        //     ineq.segment(idx, horizon) = u_w.array() - consts.ω_max;
        //     idx += horizon;
            
        //     // Angular velocity lower bounds
        //     ineq.segment(idx, horizon) = -u_w.array() - consts.ω_max;
        //     idx += horizon;
            
        //     // vs upper bounds
        //     ineq.segment(idx, horizon) = u_vs.array() - consts.v_s_max;
        //     idx += horizon;
            
        //     // vs lower bounds
        //     ineq.segment(idx, horizon) = -u_vs.array() + 0.2 * consts.v_s_max;
        //     idx += horizon;
        // });




    

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

    void publishReference() {
        auto p = std::make_unique<nav_msgs::msg::Path>();
        p->header.frame_id = "map";
        p->header.stamp = now();
        const auto& points = spline_.points();
        p->poses.reserve(points.size());

        for (const auto& pt : points) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = p->header;
            ps.pose.position.x = pt.x;
            ps.pose.position.y = pt.y;
            ps.pose.orientation.w = 1.0;
            p->poses.emplace_back(std::move(ps));
        }
        reference_pub_->publish(std::move(*p));
    }

    void publishTrackBoundaries() {
        auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

        // Clear marker
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = now();
        clear_marker.ns = "track_boundaries";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array->markers.emplace_back(std::move(clear_marker));

        // Create boundary markers
        visualization_msgs::msg::Marker left_boundary, right_boundary;

        left_boundary.header.frame_id = "map";
        left_boundary.header.stamp = now();
        left_boundary.ns = "track_boundaries";
        left_boundary.id = 0;
        left_boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
        left_boundary.action = visualization_msgs::msg::Marker::ADD;
        left_boundary.scale.x = 0.05;
        left_boundary.color.r = 1.0;
        left_boundary.color.a = 0.8;

        right_boundary = left_boundary;
        right_boundary.id = 1;
        right_boundary.color.r = 0.0;
        right_boundary.color.g = 1.0;

        const auto& trajectory_points = spline_.points();
        left_boundary.points.reserve(trajectory_points.size());
        right_boundary.points.reserve(trajectory_points.size());

        for (size_t i = 0; i < trajectory_points.size(); ++i) {
            const auto& pt = trajectory_points[i];

            double dx = 0, dy = 0;
            if (i < trajectory_points.size() - 1) {
                dx = trajectory_points[i+1].x - pt.x;
                dy = trajectory_points[i+1].y - pt.y;
            } else if (i > 0) {
                dx = pt.x - trajectory_points[i-1].x;
                dy = pt.y - trajectory_points[i-1].y;
            }

            double length = std::sqrt(dx*dx + dy*dy);
            if (length > 1e-6) {
                dx /= length;
                dy /= length;
            }

            double normal_x = -dy;
            double normal_y = dx;

            geometry_msgs::msg::Point left_point, right_point;
            left_point.x = pt.x + normal_x * pt.left_bound;
            left_point.y = pt.y + normal_y * pt.left_bound;
            left_point.z = 0.0;

            right_point.x = pt.x - normal_x * pt.right_bound;
            right_point.y = pt.y - normal_y * pt.right_bound;
            right_point.z = 0.0;

            left_boundary.points.emplace_back(std::move(left_point));
            right_boundary.points.emplace_back(std::move(right_point));
        }

        marker_array->markers.emplace_back(std::move(left_boundary));
        marker_array->markers.emplace_back(std::move(right_boundary));
        boundary_pub_->publish(std::move(*marker_array));
    }

    void initState() {
        state_.x0.setZero();
        state_.u0.setZero();
        auto start_point = spline_.getPoint(0.0);
        state_.x0 << 0, 0, 0.3, 0.0;
        state_.u0 << 0.0, 0, 0.2*constants_.v_s_max;

        auto cmd = std::make_unique<nav_msgs::msg::Odometry>();
        cmd->header.frame_id = "base_link";
        cmd->header.stamp = now();
        cmd->twist.twist.linear.x = state_.u0(0);
        cmd->twist.twist.angular.z = state_.u0(1);
        cmd->twist.twist.linear.y = state_.u0(2);
        cmd_pub_->publish(std::move(*cmd));
    }

    void publishObstacles() {
        auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
        int id = 0;

        for (size_t i = 0; i < obstacles_.size(); ++i) {
            const auto& obstacle = obstacles_[i];
            auto pos = obstacle.getCurrentPosition(spline_);
            double obs_x = pos.first;
            double obs_y = pos.second;

            // Obstacle marker
            visualization_msgs::msg::Marker obstacle_marker;
            obstacle_marker.header.frame_id = "map";
            obstacle_marker.header.stamp = now();
            obstacle_marker.ns = "obstacles_physical";
            obstacle_marker.id = id++;
            obstacle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            obstacle_marker.action = visualization_msgs::msg::Marker::ADD;

            obstacle_marker.pose.position.x = obs_x;
            obstacle_marker.pose.position.y = obs_y;
            obstacle_marker.pose.position.z = 0.25;
            obstacle_marker.pose.orientation.w = 1.0;

            obstacle_marker.scale.x = obstacle.radius * 2.0;
            obstacle_marker.scale.y = obstacle.radius * 2.0;
            obstacle_marker.scale.z = 0.5;

            obstacle_marker.color.r = 0.8;
            obstacle_marker.color.a = 0.9;

            marker_array->markers.emplace_back(std::move(obstacle_marker));

            // Safety zone
            visualization_msgs::msg::Marker safety_zone;
            safety_zone.header.frame_id = "map";
            safety_zone.header.stamp = now();
            safety_zone.ns = "obstacle_safety_zones";
            safety_zone.id = id++;
            safety_zone.type = visualization_msgs::msg::Marker::SPHERE;
            safety_zone.action = visualization_msgs::msg::Marker::ADD;

            safety_zone.pose.position.x = obs_x;
            safety_zone.pose.position.y = obs_y;
            safety_zone.pose.position.z = 0.0;
            safety_zone.pose.orientation.w = 1.0;

            double total_safety_radius = obstacle.radius + obstacle.safety_dist + constants_.vehicle_radius;
            safety_zone.scale.x = total_safety_radius * 2.0;
            safety_zone.scale.y = total_safety_radius * 2.0;
            safety_zone.scale.z = 0.1;

            safety_zone.color.r = 1.0;
            safety_zone.color.g = 0.3;
            safety_zone.color.b = 0.3;
            safety_zone.color.a = 0.3;

            marker_array->markers.emplace_back(std::move(safety_zone));
        }

        // Vehicle body marker
        visualization_msgs::msg::Marker vehicle_body;
        vehicle_body.header.frame_id = "map";
        vehicle_body.header.stamp = now();
        vehicle_body.ns = "vehicle_body";
        vehicle_body.id = id++;
        vehicle_body.type = visualization_msgs::msg::Marker::CYLINDER;
        vehicle_body.action = visualization_msgs::msg::Marker::ADD;

        vehicle_body.pose.position.x = state_.x0(0);
        vehicle_body.pose.position.y = state_.x0(1);
        vehicle_body.pose.position.z = 0.0;
        vehicle_body.pose.orientation.w = 1.0;

        vehicle_body.scale.x = constants_.vehicle_radius*2;
        vehicle_body.scale.y = constants_.vehicle_radius*2;
        vehicle_body.scale.z = 0.1;

        vehicle_body.color.b = 0.8;
        vehicle_body.color.a = 1.0;

        marker_array->markers.emplace_back(std::move(vehicle_body));
        obstacle_pub_->publish(std::move(*marker_array));
    }

    void printLapSummary() {
        if (state_.lap_times.empty()) return;

        RCLCPP_INFO(get_logger(), "\n========== LAP SUMMARY ==========");
        for (size_t i = 0; i < state_.lap_times.size(); ++i) {
            RCLCPP_INFO(get_logger(), "Lap %zu: %.3f seconds", i + 1, state_.lap_times[i]);
        }

        double total_time = std::accumulate(state_.lap_times.begin(), state_.lap_times.end(), 0.0);
        double avg_time = total_time / state_.lap_times.size();
        double best_time = *std::min_element(state_.lap_times.begin(), state_.lap_times.end());

        RCLCPP_INFO(get_logger(), "Total time: %.2f seconds", total_time);
        RCLCPP_INFO(get_logger(), "Average lap time: %.3f seconds", avg_time);
        RCLCPP_INFO(get_logger(), "Best lap time: %.3f seconds", best_time);
        RCLCPP_INFO(get_logger(), "Total distance: %.2f meters", desired_laps_ * state_.track_length);
    }


    void printErrorSummary() {
    if (contour_errors_.empty() || lag_errors_.empty() || avg_solver_times_.empty()) return;
    
    RCLCPP_INFO(get_logger(), "\n========== ERROR AND SOLVER TIME SUMMARY ==========");
    for (size_t i = 0; i < contour_errors_.size(); ++i) {
        RCLCPP_INFO(get_logger(), "Lap %zu: RMS Contour = %.4f m, RMS Lag = %.4f m, Avg Solver Time = %.6f s", 
                    i + 1, contour_errors_[i], lag_errors_[i], avg_solver_times_[i]);
    }
    
    // Calculate averages
    double avg_contour = std::accumulate(contour_errors_.begin(), contour_errors_.end(), 0.0) / contour_errors_.size();
    double avg_lag = std::accumulate(lag_errors_.begin(), lag_errors_.end(), 0.0) / lag_errors_.size();
    double total_avg_solver_time = std::accumulate(avg_solver_times_.begin(), avg_solver_times_.end(), 0.0) / avg_solver_times_.size();
    
    // Calculate min/max values
    double min_contour = *std::min_element(contour_errors_.begin(), contour_errors_.end());
    double max_contour = *std::max_element(contour_errors_.begin(), contour_errors_.end());
    double min_lag = *std::min_element(lag_errors_.begin(), lag_errors_.end());
    double max_lag = *std::max_element(lag_errors_.begin(), lag_errors_.end());
    double min_solver_time = *std::min_element(avg_solver_times_.begin(), avg_solver_times_.end());
    double max_solver_time = *std::max_element(avg_solver_times_.begin(), avg_solver_times_.end());
    
    // Print error statistics
    RCLCPP_INFO(get_logger(), "Average RMS Contour Error: %.4f m", avg_contour);
    RCLCPP_INFO(get_logger(), "Average RMS Lag Error: %.4f m", avg_lag);
    RCLCPP_INFO(get_logger(), "Best RMS Contour Error: %.4f m", min_contour);
    RCLCPP_INFO(get_logger(), "Worst RMS Contour Error: %.4f m", max_contour);
    RCLCPP_INFO(get_logger(), "Best RMS Lag Error: %.4f m", min_lag);
    RCLCPP_INFO(get_logger(), "Worst RMS Lag Error: %.4f m", max_lag);
    
    // Print solver time statistics
    RCLCPP_INFO(get_logger(), "Overall Average Solver Time: %.6f s", total_avg_solver_time);
    RCLCPP_INFO(get_logger(), "Best Average Solver Time: %.6f s", min_solver_time);
    RCLCPP_INFO(get_logger(), "Worst Average Solver Time: %.6f s", max_solver_time);
    
    // Calculate total solver time across all laps
    double total_solver_time = std::accumulate(avg_solver_times_.begin(), avg_solver_times_.end(), 0.0) * 
                               (state_.track_length / constants_.dt / constants_.rate); // Approximate solver calls per lap
    RCLCPP_INFO(get_logger(), "Estimated Total Solver Time: %.3f s", total_solver_time);
    
    // Calculate overall RMS across all laps
    double total_contour_variance = 0.0;
    double total_lag_variance = 0.0;
    for (size_t i = 0; i < contour_errors_.size(); ++i) {
        total_contour_variance += contour_errors_[i] * contour_errors_[i];
        total_lag_variance += lag_errors_[i] * lag_errors_[i];
    }
    double overall_rms_contour = std::sqrt(total_contour_variance / contour_errors_.size());
    double overall_rms_lag = std::sqrt(total_lag_variance / lag_errors_.size());
    
    RCLCPP_INFO(get_logger(), "Overall RMS Contour Error: %.4f m", overall_rms_contour);
    RCLCPP_INFO(get_logger(), "Overall RMS Lag Error: %.4f m", overall_rms_lag);
}
    // void odomCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    //     auto it = std::find(msg->name.begin(), msg->name.end(), robot_name_);
    //     if (it == msg->name.end()) {
    //         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
    //                           "Robot model '%s' not found in model_states", 
    //                           robot_name_.c_str());
    //         return;
    //     }

    //     size_t robot_index = std::distance(msg->name.begin(), it);
    //     const auto& pose = msg->pose[robot_index];

    //     // Update state with minimal operations
    //     state_.x0(0) = pose.position.x;
    //     state_.x0(1) = pose.position.y;
    //     double qw = pose.orientation.w;
    //     double qz = pose.orientation.z;
    //     state_.x0(2) = 2.0 * std::atan2(qz, qw);


 void localizationPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        auto start = std::chrono::steady_clock::now();

        state_.x0(0) = msg->pose.pose.position.x;
        state_.x0(1) = msg->pose.pose.position.y;
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        state_.x0(2) = 2.0 * std::atan2(qz, qw);

        // Calculate current errors
        auto [contour_error, lag_error] = calculateErrors();
        
        // Accumulate errors for RMS calculation
        state_.contour_error_sum += contour_error * contour_error;
        state_.lag_error_sum += lag_error * lag_error;
        state_.error_sample_count++;

        // ===== OVERTAKE DETECTION AND RECORDING =====
        // Compute DV2O for all obstacles
        if (!obstacles_.empty()) {
            for (const auto& obstacle : obstacles_) {
                auto obs_pos = obstacle.getCurrentPosition(spline_);
                double DV2O = computeV2ODistance(
                    state_.x0(0), state_.x0(1),
                    obs_pos.first, obs_pos.second,
                    obstacle.radius
                );

                double s_progress = state_.x0(3);
                
                // Start overtake recording when DV2O < 2.0
                if (!recording_overtake && DV2O < 2.0) {
                    recording_overtake = true;
                    current_dv2o.clear();
                    current_s.clear();
                    RCLCPP_INFO(get_logger(), "Overtake detected - Starting recording at s=%.2f", s_progress);
                }
                
                // Record data during overtake
                if (recording_overtake) {
                    current_dv2o.push_back(DV2O);
                    current_s.push_back(s_progress);
                    
                    // End overtake when DV2O > 2.0
                    if (DV2O > 2.0) {
                        // Find minimum DV2O and its corresponding s
                        auto min_iter = std::min_element(current_dv2o.begin(), current_dv2o.end());
                        double min_dv2o = *min_iter;
                        size_t min_index = min_iter - current_dv2o.begin();
                        double s_at_min = current_s[min_index];
                        
                        // Save overtake snippet
                        overtake_snippets.push_back({
                            current_dv2o,
                            current_s,
                            min_dv2o,
                            s_at_min
                        });
                        
                        RCLCPP_INFO(get_logger(), 
                            "Overtake %zu complete - Min DV2O: %.3f m at s=%.2f", 
                            overtake_snippets.size(), min_dv2o, s_at_min);
                        
                        recording_overtake = false;
                        
                        // Write to CSV immediately
                        writeOvertakeToCSV(overtake_snippets.size() - 1);
                    }
                }
            }
        }
        
        // Optimization
        auto before_opt = std::chrono::high_resolution_clock::now();
        auto res = controller_.optimize(state_.x0, state_.u0);
        auto seq = controller_.getOptimalSequence();
        auto after_opt = std::chrono::high_resolution_clock::now();
        double solver_time = std::chrono::duration<double>(after_opt - before_opt).count();
        
        // Accumulate solver time
        state_.solver_time_sum += solver_time;
        state_.solver_call_count++;

        // State update
        state_.x0(3) += res.cmd(2) * constants_.dt;

        // Wrap s for trajectory following
        if (multi_lap_mode_ && state_.x0(3) >= state_.track_length) {
            state_.x0(3) -= state_.track_length;
        }
        
        // LINE CROSSING LAP DETECTION
        if (detectFinishLineCrossing()) {
            state_.laps_completed++;
            double current_time = this->now().seconds();
            double lap_time = current_time - state_.lap_start_time;
            state_.lap_times.push_back(lap_time);

            RCLCPP_INFO(get_logger(),
                "Completed Lap %d/%d in %.2f seconds (heading %.3f rad)",
                state_.laps_completed, desired_laps_, lap_time, state_.x0(2));

            state_.lap_start_time = current_time;

            // Calculate RMS errors and average solver time for completed lap
            if (state_.error_sample_count > 0) {
                double rms_contour = std::sqrt(state_.contour_error_sum / state_.error_sample_count);
                double rms_lag = std::sqrt(state_.lag_error_sum / state_.error_sample_count);
                double avg_solver_time = state_.solver_time_sum / state_.solver_call_count;
                
                contour_errors_.push_back(rms_contour);
                lag_errors_.push_back(rms_lag);
                avg_solver_times_.push_back(avg_solver_time);

                RCLCPP_INFO(get_logger(),
                    "Lap %d - RMS Contour: %.4f m, RMS Lag: %.4f m, Avg Solver: %.6f s",
                    state_.laps_completed, rms_contour, rms_lag, avg_solver_time);

                // Reset accumulators for next lap
                state_.contour_error_sum = 0.0;
                state_.lag_error_sum = 0.0;
                state_.error_sample_count = 0;
                state_.solver_time_sum = 0.0;
                state_.solver_call_count = 0;
            }

            if (state_.laps_completed >= desired_laps_) {
                RCLCPP_INFO(get_logger(), "ALL %d LAPS COMPLETED!", desired_laps_);
                printErrorSummary();
                printLapSummary();
                
                geometry_msgs::msg::Twist stop_cmd;
                vel_pub_->publish(stop_cmd);
                rclcpp::shutdown();
                return;
            }
        }
        
        // Update previous position for next iteration
        state_.prev_x = state_.x0(0);
        state_.prev_y = state_.x0(1);
        state_.first_position_set = true;

        // Update obstacles
        for (auto& obstacle : obstacles_) {
            obstacle.update(constants_.dt, state_.track_length);
        }
        
        // Display lap progress using s-coordinate
        if (state_.laps_completed < desired_laps_) {
            double lap_progress = (state_.x0(3) / state_.track_length) * 100.0;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                "Lap %d/%d: %.1f%% (x:%.2f, y:%.2f, heading:%.3f, s:%.2f, Solver: %.6f s)",
                state_.laps_completed + 1, desired_laps_, lap_progress,
                state_.x0(0), state_.x0(1), state_.x0(2), state_.x0(3), solver_time);
        }
        
        // Publish results (optimized)
        publishOptimalSequences(seq);
        publishReferencePoint();
        publishTrackingError();
        publishCommands(res.cmd, solver_time);
        publishObstacles();
        updateActualPath();

        state_.u0 = res.cmd;
    }


    // Helper methods for publishing (optimized)
    void publishOptimalSequences(const auto& seq) {
        std_msgs::msg::Float64MultiArray sm, um;
        sm.data.reserve(seq.state.size());
        um.data.reserve(seq.input.size());

        for (int i = 0; i < seq.state.size(); ++i) sm.data.push_back(seq.state(i));
        for (int i = 0; i < seq.input.size(); ++i) um.data.push_back(seq.input(i));

        states_pub_->publish(sm);
        inputs_pub_->publish(um);
    }

    void publishReferencePoint() {
        auto ref = spline_.getPoint(state_.x0(3));
        geometry_msgs::msg::PointStamped ref_point;
        ref_point.header.frame_id = "map";
        ref_point.header.stamp = now();
        ref_point.point.x = ref.x;
        ref_point.point.y = ref.y;
        ref_point.point.z = 0.0;
        reference_point_pub_->publish(ref_point);
    }

    void publishTrackingError() {
        auto ref = spline_.getPoint(state_.x0(3));
        double te = fastHypot(state_.x0(0) - ref.x, state_.x0(1) - ref.y);
        std_msgs::msg::Float64 err;
        err.data = te;
        error_pub_->publish(err);
    }

    void publishCommands(const Eigen::Vector<double, Tnu>& cmd, double opt_time) {
        // Publish optimization time
        std_msgs::msg::Float64 opt_time_msg;
        opt_time_msg.data = opt_time;
        opt_time_pub_->publish(opt_time_msg);

        // Publish MPCC command
        nav_msgs::msg::Odometry cmd_msg;
        cmd_msg.header.frame_id = "base_link";
        cmd_msg.header.stamp = now();
        cmd_msg.twist.twist.linear.x = cmd(0);
        cmd_msg.twist.twist.angular.z = cmd(1);
        cmd_msg.twist.twist.linear.y = cmd(2);
        cmd_msg.twist.twist.linear.z = constants_.dt;
        cmd_msg.twist.twist.angular.x = state_.x0(3);
        cmd_pub_->publish(cmd_msg);

        // Publish velocity command
        geometry_msgs::msg::Twist tv;
        tv.linear.x = cmd(0);
        tv.angular.z = cmd(1);
        vel_pub_->publish(tv);
    }
        // Check if two line segments intersect
        bool lineSegmentsIntersect(double x1, double y1, double x2, double y2,
                                double x3, double y3, double x4, double y4) const {
            // Line segment 1: (x1,y1) to (x2,y2)
            // Line segment 2: (x3,y3) to (x4,y4)
            
            double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            
            if (std::abs(denom) < 1e-10) {
                return false; // Lines are parallel
            }
            
            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
            double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
            
            // Check if intersection point is within both line segments
            return (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0);
        }
    
        // Check if vehicle heading is in valid range for lap crossing
        bool isValidCrossingHeading(double heading) const {
            // const double TARGET_HEADING = 0.19023585334140491;  // Expected heading at finish
            const double TARGET_HEADING = 0.3;  // Expected heading at finish aut centreline

            const double HEADING_TOLERANCE = 0.4;  // Allow ±0.4 radians for overtaking
            
            // Normalize heading to [-π, π]
            double normalized_heading = std::atan2(std::sin(heading), std::cos(heading));
            
            // Check if heading is within acceptable range
            double heading_diff = std::abs(normalized_heading - TARGET_HEADING);
            
            // Also check wrapped difference (for angles near ±π)
            double wrapped_diff = std::abs(heading_diff - 2.0 * M_PI);
            heading_diff = std::min(heading_diff, wrapped_diff);
            
            return heading_diff <= HEADING_TOLERANCE;
        }
    
        // Detect finish line crossing
        bool detectFinishLineCrossing() const {
             double current_time = this->now().seconds();
            double lap_time = current_time - state_.lap_start_time;
            
            // Reject laps shorter than 5 seconds
            if (lap_time < 5.0 && state_.laps_completed > 0) {
                RCLCPP_WARN(get_logger(), "Rejecting false lap detection (%.2fs)", lap_time);
                return false;  // Skip this false detection
            }
            
            if (!state_.first_position_set) {
                return false;
            }
            
            // Get finish line coordinates (at s=0)
            auto finish_point = spline_.getPoint(0.0);
            const auto& deriv = spline_.getDerivatives(0.0);
            
            // Create finish line segment perpendicular to track direction
            const double LINE_WIDTH = 3.0;  // Width of finish line (meters)
            
            double normal_x = -std::sin(deriv.theta);
            double normal_y = std::cos(deriv.theta);
            
            double finish_x1 = finish_point.x - normal_x * LINE_WIDTH;
            double finish_y1 = finish_point.y - normal_y * LINE_WIDTH;
            double finish_x2 = finish_point.x + normal_x * LINE_WIDTH;
            double finish_y2 = finish_point.y + normal_y * LINE_WIDTH;
            
            // Check if vehicle path crossed the finish line
            bool crossed = lineSegmentsIntersect(
                state_.prev_x, state_.prev_y,           // Previous position
                state_.x0(0), state_.x0(1),             // Current position
                finish_x1, finish_y1,                   // Finish line point 1
                finish_x2, finish_y2                    // Finish line point 2
            );
            
            if (crossed) {
                // Validate heading direction
                return isValidCrossingHeading(state_.x0(2));
            }
            
            return false;
        }

        void writeOvertakeToCSV(size_t overtake_index) {
        const auto& snippet = overtake_snippets[overtake_index];
        
        // Open or create CSV file
        std::ofstream csv;
        if (overtake_index == 0) {
            csv.open("overtakes.csv", std::ios::out);
            csv << "overtake_index,min_dv2o,s_at_min,num_samples\n";
        } else {
            csv.open("overtakes.csv", std::ios::app);
        }
        
        // Write summary line
        csv << overtake_index << ","
            << snippet.min_dv2o << ","
            << snippet.s_at_min << ","
            << snippet.dv2o_snippet.size() << "\n";
        csv.close();
        
        // Write detailed snippet data to separate file
        std::string detail_filename = "overtake_" + std::to_string(overtake_index) + "_detail.csv";
        std::ofstream detail_csv(detail_filename);
        detail_csv << "s,dv2o\n";
        
        for (size_t i = 0; i < snippet.s_snippet.size(); ++i) {
            detail_csv << snippet.s_snippet[i] << "," << snippet.dv2o_snippet[i] << "\n";
        }
        detail_csv.close();
        
        RCLCPP_INFO(get_logger(), "Saved overtake %zu to %s", overtake_index, detail_filename.c_str());
    }



    void updateActualPath() {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = now();
        ps.pose.position.x = state_.x0(0);
        ps.pose.position.y = state_.x0(1);
        ps.pose.orientation.z = state_.x0(2);
        ps.pose.orientation.w = 1.0;
        actual_path_.emplace_back(std::move(ps));

        auto act = std::make_unique<nav_msgs::msg::Path>();
        act->header.frame_id = "map";
        act->header.stamp = now();
        act->poses = actual_path_;
        actual_pub_->publish(std::move(*act));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCCController>());
    rclcpp::shutdown();
    return 0;
}