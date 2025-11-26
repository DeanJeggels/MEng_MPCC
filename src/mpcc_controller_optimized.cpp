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

constexpr int Tnx = 4; // [x, y, psi, s]
constexpr int Tnu = 3; // [v, ω, v_s]
constexpr int Tny = 4;
constexpr int Tph = 3;
constexpr int Tch = 3;
constexpr int iterations = 1000;
constexpr int TRACK_BD_CONSTR = 0*(Tph+1);
constexpr int OBSTACLE_CONSTR = 0*(Tph+1);
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

class LinearSpline {
private:
    std::vector<TrajectoryPoint> trajectory_;
    std::vector<double> s_;
    std::vector<PathDerivatives> derivatives_; // Pre-computed derivatives
    double total_length_ = 0.0;
    
    // Cache for last lookup (spatial locality optimization)
    mutable size_t last_index_ = 0;
    
public:
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
        if (s >= s_.back()) return {trajectory_.back().left_bound, trajectory_.back().right_bound};
        
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

class MPCCController : public rclcpp::Node {
private:
    // Constants grouped together
    struct Constants {
        const double v_max, ω_max, v_s_max, rate, dt;
        const double vehicle_radius = 0.6;
        const double safety_distance = 0.8;
        const double obstacle_radius = 0.6;
        const double k_v = 0.1, k_ω = 5.0, k_vs = 0.1;
        const double k_c = 50.0, k_l = 50.0, k_th = 10000.0;
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

    // Grouped member variables for better cache locality
    struct ControllerState {
        Eigen::Vector<double, Tnx> x0;
        Eigen::Vector<double, Tnu> u0;
        double track_length = 0.0;
        int laps_completed = 0;
        double lap_start_time = 0.0;
    } state_;

    // Controller parameters
    double v_max_, ω_max_, v_s_max_, rate_, dt_;
    Constants constants_;

    // Configuration
    int desired_laps_ = 1;
    bool multi_lap_mode_ = true;
    std::vector<double> lap_times_;

    // Obstacle data (use pointers to avoid copies)
    nav_msgs::msg::Path::SharedPtr current_obstacle_path_;
    nav_msgs::msg::Path::SharedPtr predicted_obstacle_path_;
    geometry_msgs::msg::PointStamped::SharedPtr current_obstacle_point_;

    LinearSpline spline_;
    std::vector<geometry_msgs::msg::PoseStamped> actual_path_;

    // Publishers and Subscribers - grouped for better memory layout
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_, actual_pub_, predicted_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr states_pub_, inputs_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_, opt_time_pub_, dv2o_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr reference_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundary_pub_, obstacle_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr obstacle_actual_sub_, obstacle_predicted_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_current_sub_;

public:
    MPCCController()
    : Node("mpcc_controller"),
      v_max_(0.2), ω_max_(PI_2), v_s_max_(0.2), rate_(5.0), dt_(1.0/rate_),
      constants_(v_max_, ω_max_, v_s_max_, rate_)
    {
        // Pre-allocate vectors
        lap_times_.reserve(desired_laps_);
        actual_path_.reserve(10000); // Pre-allocate for performance
        
        if (!spline_.loadFromCSV("/home/deanjeggs/new_ros2_ws/src/mpcc/racetracks/recorded_path_straight.csv")) {
            RCLCPP_ERROR(get_logger(), "Failed loading spline");
            rclcpp::shutdown();
            return;
        }

        state_.track_length = spline_.getTotalLength();
        state_.lap_start_time = this->now().seconds();
        
        // Use more efficient logging (avoid string formatting in hot path)
        RCLCPP_INFO_ONCE(get_logger(), "Track length: %.2f meters, Target laps: %d", 
                         state_.track_length, desired_laps_);

        setupSubscribers();
        setupPublishers();
        publishReference();
        // publishTrackBoundaries();
        setupController();
        initState();
    }

private:
    void setupSubscribers() {
        using std::placeholders::_1;
        localization_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization_pose", 10,
            std::bind(&MPCCController::localizationPoseCallback, this, _1));

        // obstacle_actual_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        //     "/mpcc_obst/actual_path", 10,
        //     std::bind(&MPCCController::obstacleActualPathCallback, this, _1));

        // obstacle_predicted_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        //     "/mpcc_obst/predicted_path", 10,
        //     std::bind(&MPCCController::obstaclePredictedPathCallback, this, _1));

        // obstacle_current_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        //     "/mpcc_obst/current_reference_point", 10,
        //     std::bind(&MPCCController::obstacleCurrentPointCallback, this, _1));
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
        dv2o_pub_ = create_publisher<std_msgs::msg::Float64>("mpcc/dv2o_distance", 10);
    }

    // Inline frequently called functions
    inline double fastHypot(double x, double y) const {
        return std::sqrt(x*x + y*y);
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
        } else if (DV2O <= DSft_O) {
            double ratio = DV2O / DSft_O;
            return constants_.Pk * std::exp(-2.0 * ratio * ratio);
        }
        return 0.0;
    }

    void printLapSummary() {
        if (lap_times_.empty()) return;
        
        RCLCPP_INFO(get_logger(), "\n========== LAP SUMMARY ==========");
        for (size_t i = 0; i < lap_times_.size(); ++i) {
            RCLCPP_INFO(get_logger(), "Lap %zu: %.3f seconds", i + 1, lap_times_[i]);
        }
        
        double total_time = std::accumulate(lap_times_.begin(), lap_times_.end(), 0.0);
        double avg_time = total_time / lap_times_.size();
        double best_time = *std::min_element(lap_times_.begin(), lap_times_.end());
        
        RCLCPP_INFO(get_logger(), "Total time: %.2f seconds", total_time);
        RCLCPP_INFO(get_logger(), "Average lap time: %.3f seconds", avg_time);
        RCLCPP_INFO(get_logger(), "Best lap time: %.3f seconds", best_time);
        RCLCPP_INFO(get_logger(), "Total distance: %.2f meters", desired_laps_ * state_.track_length);
    }

    // Callback functions - use move semantics where possible
    void obstacleActualPathCallback(nav_msgs::msg::Path::SharedPtr msg) {
        current_obstacle_path_ = std::move(msg);
    }

    void obstaclePredictedPathCallback(nav_msgs::msg::Path::SharedPtr msg) {
        predicted_obstacle_path_ = std::move(msg);
    }

    void obstacleCurrentPointCallback(geometry_msgs::msg::PointStamped::SharedPtr msg) {
        current_obstacle_point_ = std::move(msg);
    }

    void publishReference() {
        auto p = std::make_unique<nav_msgs::msg::Path>();
        p->header.frame_id = "map";
        p->header.stamp = now();
        
        const auto& points = spline_.points();
        p->poses.reserve(points.size()); // Pre-allocate
        
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

    void setupController() {
        controller_.setLoggerLevel(mpc::Logger::LogLevel::NORMAL);
        
        // Capture constants by reference to avoid copies
        const auto& consts = constants_;
        
        controller_.setIneqConFunction(
        [this, &consts](mpc::cvec<Tineq>& ineq, const mpc::mat<Tph + 1, Tnx>& x, 
                       const mpc::mat<Tph + 1, Tny>&, const mpc::mat<Tph + 1, Tnu>& u, const double&) {
            int idx = 0;
            
            for (int i = 0; i <= Tph; ++i) {
                // Input constraints
                ineq(idx++) = u(i,0) - consts.v_max;
                ineq(idx++) = -u(i,0) + 0.5*consts.v_max;
                ineq(idx++) = u(i,1) - consts.ω_max;
                ineq(idx++) = -u(i,1) - consts.ω_max;
                ineq(idx++) = u(i,2) - consts.v_s_max;
                ineq(idx++) = -u(i,2) + 0.5*consts.v_s_max;

                // Track boundary constraints
                double s = x(i,3);
                auto ref = spline_.getPoint(s);
                const auto& deriv = spline_.getDerivatives(s);
                
                double dx = x(i,0) - ref.x;
                double dy = x(i,1) - ref.y;
                double e_l = -std::cos(deriv.theta)*dx - std::sin(deriv.theta)*dy;

                // auto [left_bd, right_bd] = spline_.getBoundaries(s);
                // ineq(idx++) = e_l - (left_bd - consts.vehicle_radius);
                // ineq(idx++) = -e_l - (right_bd - consts.vehicle_radius);
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

        // Optimized cost function
        controller_.setObjectiveFunction(
            [this, &consts](const mpc::mat<Tph + 1, Tnx>& X, const mpc::mat<Tph + 1, Tny>&, 
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
            
                    // Obstacle avoidance (optimized)
                    if (current_obstacle_point_ && predicted_obstacle_path_) {
                        double obs_x, obs_y;
                        
                        if (i == 0) {
                            obs_x = current_obstacle_point_->point.x;
                            obs_y = current_obstacle_point_->point.y;
                        } else if (static_cast<size_t>(i-1) < predicted_obstacle_path_->poses.size()) {
                            const auto& pose = predicted_obstacle_path_->poses[i-1].pose;
                            obs_x = pose.position.x;
                            obs_y = pose.position.y;
                        } else if (!predicted_obstacle_path_->poses.empty()) {
                            const auto& last_pose = predicted_obstacle_path_->poses.back().pose;
                            obs_x = last_pose.position.x;
                            obs_y = last_pose.position.y;
                        } else {
                            continue;
                        }
                        
                        double DV2O = computeV2ODistance(X(i,0), X(i,1), obs_x, obs_y, consts.obstacle_radius);
                        double qV2O = getAdaptiveObstacleWeight(DV2O, consts.safety_distance);
                        
                        if (qV2O > 0) {
                            double eV2O = DV2O - consts.safety_distance;
                            cost += qV2O * eV2O * eV2O;
                        }
                    }
                    
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

    void publishTrackBoundaries() {
        auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
        
        // Clear marker
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = now();
        clear_marker.ns = "track_boundaries";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array->markers.emplace_back(std::move(clear_marker));
        
        // Create boundary markers with pre-allocation
        visualization_msgs::msg::Marker left_boundary, right_boundary;
        
        // Setup left boundary
        left_boundary.header.frame_id = "map";
        left_boundary.header.stamp = now();
        left_boundary.ns = "track_boundaries";
        left_boundary.id = 0;
        left_boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
        left_boundary.action = visualization_msgs::msg::Marker::ADD;
        left_boundary.scale.x = 0.05;
        left_boundary.color.r = 1.0;
        left_boundary.color.a = 0.8;
        
        // Setup right boundary
        right_boundary = left_boundary;
        right_boundary.id = 1;
        right_boundary.color.r = 0.0;
        right_boundary.color.g = 1.0;
        
        const auto& trajectory_points = spline_.points();
        left_boundary.points.reserve(trajectory_points.size());
        right_boundary.points.reserve(trajectory_points.size());
        
        for (size_t i = 0; i < trajectory_points.size(); ++i) {
            const auto& pt = trajectory_points[i];
            const auto& deriv = spline_.getDerivatives(state_.track_length * i / trajectory_points.size());
            
            double normal_x = -std::sin(deriv.theta);
            double normal_y = std::cos(deriv.theta);
            
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
        state_.x0 << 0, 0.0, 0, 0;
        state_.u0 << 0.0, 0, 0.5*constants_.v_s_max;

        // Publish initial command
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
        
        // Clear previous markers first
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = now();
        clear_marker.ns = "obstacles_from_topics";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array->markers.emplace_back(std::move(clear_marker));
        
        // Clear vehicle body namespace too
        visualization_msgs::msg::Marker clear_vehicle;
        clear_vehicle.header.frame_id = "map";
        clear_vehicle.header.stamp = now();
        clear_vehicle.ns = "vehicle_body";
        clear_vehicle.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array->markers.emplace_back(std::move(clear_vehicle));
        
        if (current_obstacle_point_) {
            double obs_x = current_obstacle_point_->point.x;
            double obs_y = current_obstacle_point_->point.y;
            
            // Obstacle marker
            visualization_msgs::msg::Marker obstacle_marker;
            obstacle_marker.header.frame_id = "map";
            obstacle_marker.header.stamp = now();
            obstacle_marker.ns = "obstacles_from_topics";
            obstacle_marker.id = 100;
            obstacle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            obstacle_marker.action = visualization_msgs::msg::Marker::ADD;
            
            obstacle_marker.pose.position.x = obs_x;
            obstacle_marker.pose.position.y = obs_y;
            obstacle_marker.pose.position.z = 0.25;
            obstacle_marker.pose.orientation.w = 1.0;
            
            obstacle_marker.scale.x = constants_.obstacle_radius * 2.0;
            obstacle_marker.scale.y = constants_.obstacle_radius * 2.0;
            obstacle_marker.scale.z = 0.5;
            
            obstacle_marker.color.r = 0.8;
            obstacle_marker.color.a = 0.9;
            
            marker_array->markers.emplace_back(std::move(obstacle_marker));
            
            // Safety zone
            visualization_msgs::msg::Marker safety_zone;
            safety_zone.header.frame_id = "map";
            safety_zone.header.stamp = now();
            safety_zone.ns = "obstacle_safety_zones";
            safety_zone.id = 101;
            safety_zone.type = visualization_msgs::msg::Marker::SPHERE;
            safety_zone.action = visualization_msgs::msg::Marker::ADD;
            
            safety_zone.pose.position.x = obs_x;
            safety_zone.pose.position.y = obs_y;
            safety_zone.pose.position.z = 0.0;
            safety_zone.pose.orientation.w = 1.0;
            
            double total_safety_radius = constants_.obstacle_radius + constants_.safety_distance + constants_.vehicle_radius;
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
        vehicle_body.id = 200;
        vehicle_body.type = visualization_msgs::msg::Marker::CYLINDER;
        vehicle_body.action = visualization_msgs::msg::Marker::ADD;
        
        vehicle_body.pose.position.x = state_.x0(0);
        vehicle_body.pose.position.y = state_.x0(1);
        vehicle_body.pose.position.z = 0.0;
        vehicle_body.pose.orientation.w = 1.0;
        
        vehicle_body.scale.x = constants_.vehicle_radius * 2;
        vehicle_body.scale.y = constants_.vehicle_radius * 2;
        vehicle_body.scale.z = 0.1;
        
        vehicle_body.color.b = 0.8;
        vehicle_body.color.a = 1.0;
        
        marker_array->markers.emplace_back(std::move(vehicle_body));
        obstacle_pub_->publish(std::move(*marker_array));
    }

    void localizationPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Update state with minimal operations
        state_.x0(0) = msg->pose.pose.position.x;
        state_.x0(1) = msg->pose.pose.position.y;
        
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        state_.x0(2) = 2.0 * std::atan2(qz, qw);

        // DV2O calculation and publishing
        if (current_obstacle_point_) {
            double obs_x = current_obstacle_point_->point.x;
            double obs_y = current_obstacle_point_->point.y;
            
            double current_dv2o = computeV2ODistance(state_.x0(0), state_.x0(1), obs_x, obs_y, constants_.obstacle_radius);
            
            std_msgs::msg::Float64 dv2o_msg;
            dv2o_msg.data = current_dv2o;
            dv2o_pub_->publish(dv2o_msg);
        }

        // Optimization
        auto before_opt = std::chrono::high_resolution_clock::now();
        auto res = controller_.optimize(state_.x0, state_.u0);
        auto seq = controller_.getOptimalSequence();
        auto opt_time = (std::chrono::high_resolution_clock::now() - before_opt).count() / 1e9;

        // State update
        state_.x0(3) += res.cmd(2) * constants_.dt;

        // Multi-lap handling
        if (multi_lap_mode_ && state_.x0(3) >= state_.track_length) {
            state_.laps_completed++;
            state_.x0(3) -= state_.track_length;

            double current_time = this->now().seconds();
            double lap_time = current_time - state_.lap_start_time;
            lap_times_.push_back(lap_time);
            
            RCLCPP_INFO(get_logger(), "🏁 Completed Lap %d/%d in %.2f seconds", 
                        state_.laps_completed, desired_laps_, lap_time);
            
            state_.lap_start_time = current_time;
            
            if (state_.laps_completed >= desired_laps_) {
                RCLCPP_INFO(get_logger(), "🎉 ALL %d LAPS COMPLETED! 🎉", desired_laps_);
                printLapSummary();
                
                geometry_msgs::msg::Twist stop_cmd;
                vel_pub_->publish(stop_cmd);
                rclcpp::shutdown();
                return;
            }
        }

        // Publish results (optimized with move semantics)
        publishPredictedPath(seq);
        publishOptimalSequences(seq);
        publishReferencePoint();
        publishTrackingError();
        publishCommands(res.cmd, opt_time);
        publishObstacles();
        
        // Update actual path
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

        state_.u0 = res.cmd;
    }

    // Helper methods for publishing (optimized)
    void publishPredictedPath(const auto& seq) {
        auto predicted_path = std::make_unique<nav_msgs::msg::Path>();
        predicted_path->header.frame_id = "map";
        predicted_path->header.stamp = now();
        predicted_path->poses.reserve(Tph);
        
        for (int i = 1; i <= Tph; ++i) {
            geometry_msgs::msg::PoseStamped predicted_pose;
            predicted_pose.header = predicted_path->header;
            predicted_pose.pose.position.x = seq.state(i, 0);
            predicted_pose.pose.position.y = seq.state(i, 1);
            predicted_pose.pose.position.z = 0.0;
            
            double psi = seq.state(i, 2);
            predicted_pose.pose.orientation.z = std::sin(psi/2.0);
            predicted_pose.pose.orientation.w = std::cos(psi/2.0);
            
            predicted_path->poses.emplace_back(std::move(predicted_pose));
        }
        
        predicted_pub_->publish(std::move(*predicted_path));
    }

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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCCController>());
    rclcpp::shutdown();
    return 0;
}
