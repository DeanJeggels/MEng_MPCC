
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <mpc/NLMPC.hpp>
#include <mpc/Utils.hpp>  
#include <mpc/IMPC.hpp>
#include <mpc/Logger.hpp>
#include <chrono>

constexpr int Tnx   = 4;   // [x, y, psi, s] - vehicle states only
constexpr int Tnu   = 3;   // [v, ω, v_s]
constexpr int Tny   = 4;   // same as states
constexpr int Tph   = 3;   // prediction horizon
constexpr int Tch   = 3;   // control horizon
constexpr int iterations = 1000;
constexpr int MAX_OBSTACLES = 5;  // Maximum number of obstacles for constraint sizing
// constexpr int Tineq = 6*(Tph+1) + MAX_OBSTACLES*(Tph+1); // Input constraints + obstacle constraints
// constexpr int Tineq = 6*(Tph+1); // Input constraints only
constexpr int TRACK_BD_CONSTR = 2*(Tph+1);
constexpr int Tineq = 6*(Tph+1) + TRACK_BD_CONSTR; // input + track boundaries
constexpr int Teq   = 0;

mpc::NLMPC<Tnx,Tnu,Tny,Tph,Tch,Tineq,Teq> controller_;

struct TrajectoryPoint { 
    double x, y; 
    double left_bound, right_bound; 
};

struct PathDerivatives { double dx_ds, dy_ds; };



class LinearSpline {
public:
    bool loadFromCSV(const std::string &file) {
        std::ifstream in(file);
        if (!in) return false;
        trajectory_.clear();
        s_.clear();

        std::string line;
        std::getline(in, line); // skip header
        while (std::getline(in, line)) {
            std::stringstream ss(line);
            TrajectoryPoint pt;
            if (!(ss >> pt.x)) break;
            if (ss.peek() == ',') ss.ignore();
            if (!(ss >> pt.y)) break;
            if (ss.peek() == ',') ss.ignore();
            if (!(ss >> pt.right_bound)) break;  // Column 3 → right_bound
            if (ss.peek() == ',') ss.ignore();
            if (!(ss >> pt.left_bound)) break;   // Column 4 → left_bound
            trajectory_.push_back(pt);
        }
        if (trajectory_.empty()) return false;

        s_.resize(trajectory_.size());
        s_[0] = 0;
        for (size_t i = 1; i < trajectory_.size(); ++i) {
            auto &a = trajectory_[i-1], &b = trajectory_[i];
            double ds = std::hypot(b.x - a.x, b.y - a.y);
            s_[i] = s_[i-1] + ds;
        }
        return true;
    }

TrajectoryPoint getPoint(double s) const {

    if (trajectory_.empty()) return {0.0, 0.0, 0.0, 0.0};
    if (s <= s_.front()) return trajectory_.front();
    if (s >= s_.back())  return trajectory_.back();

    auto it = std::lower_bound(s_.begin(), s_.end(), s);
    size_t i = std::max<size_t>(0, (it - s_.begin()) - 1);
    double t = (s - s_[i]) / (s_[i+1] - s_[i]);
    const auto &A = trajectory_[i], &B = trajectory_[i+1];
    
    return { 
      A.x + t*(B.x - A.x), 
      A.y + t*(B.y - A.y),
      A.left_bound + t*(B.left_bound - A.left_bound),
      A.right_bound + t*(B.right_bound - A.right_bound)
    };
  }


  PathDerivatives getDerivatives(double s) const {
    if (trajectory_.size() < 2) return {1,0};
    if (s <= s_.front()) s = s_.front();
    if (s >= s_.back())  s = s_.back();
    auto it = std::lower_bound(s_.begin(), s_.end(), s);
    size_t i = std::min<size_t>(trajectory_.size()-2, std::max<size_t>(0, (it - s_.begin()) - 1));
    double ds = s_[i+1] - s_[i];
    if (ds < 1e-6) return {1,0};
    const auto &A = trajectory_[i], &B = trajectory_[i+1];
    return {(B.x - A.x)/ds, (B.y - A.y)/ds};
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

private:
  std::vector<TrajectoryPoint> trajectory_;
  std::vector<double> s_;
};

// Obstacle struct definition
struct Obstacle {
    double s;                    // Current position along trajectory (instead of x,y)
    double vs;                   // Speed along trajectory (m/s)
    double radius;               // Obstacle radius
    double safety_dist;          // Safety distance
    int id;                      // Unique identifier
    double confidence;           
    
    // Predict future trajectory position at time step i
    std::pair<double, double> predictPosition(double dt, int steps, const LinearSpline& spline) const {
        double future_s = s + vs * dt * steps;  // Future s position
        auto future_point = spline.getPoint(future_s);
        return {future_point.x, future_point.y + 0.1};
    }
    
    // Update obstacle state along trajectory
    void update(double dt) {
        s += vs * dt;  // Move along trajectory
    }
    
    // Get current x,y position from trajectory
    std::pair<double, double> getCurrentPosition(const LinearSpline& spline) const {
        auto current_point = spline.getPoint(s);
        return {current_point.x, current_point.y+0.1};
    }
};


class MPCCController : public rclcpp::Node {
public:
  MPCCController()
  : Node("mpcc_controller"),
    v_max_(0.1), ω_max_(M_PI/2), v_s_max_(0.1), rate_(10.0), dt_(1.0/rate_)
  {
    if (!spline_.loadFromCSV("/home/deanjeggs/new_ros2_ws/src/mpcc/racetracks/aut_centerline.csv")) {
      RCLCPP_ERROR(get_logger(), "Failed loading spline");
      rclcpp::shutdown();
      return;
    }

    using std::placeholders::_1;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/karretjie/odom", 10,
      std::bind(&MPCCController::odomCallback, this, _1));

    reference_pub_ = create_publisher<nav_msgs::msg::Path>("/mpcc/reference_path_karretjie", 10);
    predicted_pub_ = create_publisher<nav_msgs::msg::Path>("/mpcc/predicted_path_karretjie", 10);
    actual_pub_ = create_publisher<nav_msgs::msg::Path>("/mpcc/actual_path/karretjie", 10);
    cmd_pub_ = create_publisher<nav_msgs::msg::Odometry>("/mpcc/result_cmd_karretjie", 10);
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_karretjie", 10);
    obstacle_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mpcc/obstacles_karretjie", 10);
    reference_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/mpcc/current_reference_point", 10);
    boundary_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mpcc/track_boundaries_karretjie", 10);

    publishReference();
    publishTrackBoundaries();
    setupController();
    setupObstacles();
    initState();
  }

private:
  void publishReference() {
    nav_msgs::msg::Path p;
    p.header.frame_id = "map";
    p.header.stamp = now();
    for (auto &pt : spline_.points()) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = p.header;
      ps.pose.position.x = pt.x;
      ps.pose.position.y = pt.y;
      ps.pose.orientation.w = 1.0;
      p.poses.push_back(ps);
    }
    reference_pub_->publish(p);
  }

  void setupController() {
    controller_.setLoggerLevel(mpc::Logger::LogLevel::NORMAL);
    
    controller_.setIneqConFunction(
      [this](mpc::cvec<Tineq>& ineq, const mpc::mat<Tph + 1, Tnx>& x, const mpc::mat<Tph + 1, Tny>&, const mpc::mat<Tph + 1, Tnu>& u, const double&) {
        int idx = 0;

        for (int i = 0; i <= Tph; ++i) {
          // Input constraints
          ineq(idx++) = u(i,0) - v_max_;
          ineq(idx++) = -u(i,0) + 0.2*v_max_;
          // ineq(idx++) = -u(i,0);
          ineq(idx++) = u(i,1) - ω_max_;
          ineq(idx++) = -u(i,1) - ω_max_;
          ineq(idx++) = u(i,2) - v_s_max_;
          ineq(idx++) = -u(i,2) + 0.2*v_s_max_;
          // ineq(idx++) = -u(i,2);
          double s = x(i,3);
          auto ref = spline_.getPoint(s);
          auto deriv = spline_.getDerivatives(s);
          double theta = std::atan2(deriv.dy_ds, deriv.dx_ds);
          double dx = x(i,0) - ref.x;
          double dy = x(i,1) - ref.y;
          double e_l = -std::cos(theta)*dx - std::sin(theta)*dy;

          auto [left_bd, right_bd] = spline_.getBoundaries(s);

          // Left boundary: e_l <= left_bd - vehicle_radius
          ineq(idx++) = e_l - (left_bd - vehicle_radius_);
          
          // Right boundary: -e_l <= right_bd - vehicle_radius  
          ineq(idx++) = -e_l - (right_bd - vehicle_radius_);
         
          // Obstacle avoidance constraints
          // for (const auto& obstacle : obstacles_) {
          //   auto pred_pos = obstacle.predictPosition(dt_, i);
          //   double dx = x(i,0) - pred_pos.first;   // x_vehicle - x_obstacle_predicted
          //   double dy = x(i,1) - pred_pos.second;  // y_vehicle - y_obstacle_predicted
          //   double distance = sqrt(dx*dx + dy*dy);
          //   double DV2O = distance - obstacle.radius - vehicle_radius_;
          //   ineq(idx++) = DV2O - obstacle.safety_dist;  // DV2O >= safety_dist
          // }
          
          // // Fill remaining obstacle constraints with large positive values (inactive)
          // for (size_t j = obstacles_.size(); j < MAX_OBSTACLES; ++j) {
          //   ineq(idx++) = 1000.0;  // Large positive value = constraint satisfied
          // }
        }
      }, 1e-6);

    controller_.setStateSpaceFunction(
      [this](mpc::cvec<Tnx>& x_next, const mpc::cvec<Tnx>& x, const mpc::cvec<Tnu>& u, const unsigned int&){
        double dt = 1.0/rate_;
        
        x_next(0) = x(0) + u(0)*std::cos(x(2))*dt;  // x
        x_next(1) = x(1) + u(0)*std::sin(x(2))*dt;  // y
        x_next(2) = x(2) + u(1)*dt;                 // psi
        x_next(3) = x(3) + u(2)*dt;                 // s
      });

    controller_.setOutputFunction(
      [](auto &y, const auto &x, const auto&, unsigned){ y = x; });

    // controller_.setObjectiveFunction(
    //   [this](const mpc::mat<Tph + 1, Tnx>& X, const mpc::mat<Tph + 1, Tny>&, const mpc::mat<Tph + 1, Tnu>& U, const double&){
    //     double cost = 0, prev_v=0, prev_ω=0, prev_vs=0;
        
    //     for (int i = 0; i <= Tph; ++i) {
    //       // Standard MPCC tracking costs
    //       double s = X(i,3);
    //       auto ref = spline_.getPoint(s);
    //       auto d   = spline_.getDerivatives(s);
    //       double θ = std::atan2(d.dy_ds, d.dx_ds);
    //       double e_c =  std::sin(θ)*(X(i,0)-ref.x) - std::cos(θ)*(X(i,1)-ref.y);
    //       double e_l = -std::cos(θ)*(X(i,0)-ref.x) - std::sin(θ)*(X(i,1)-ref.y);
    //       Eigen::Vector2d err(e_c,e_l);
    //       cost += err.transpose()*Q_*err;
    //       cost += -qθ_*s;
        
    //       // // Obstacle avoidance cost with Gaussian weight
    //       for (const auto& obstacle : obstacles_) {
    //         auto pred_pos = obstacle.predictPosition(dt_, i, spline_);  // Pass spline reference
    //         double DV2O = computeV2ODistance(X(i,0), X(i,1), pred_pos.first, pred_pos.second, obstacle.radius);
    //         double qV2O = getAdaptiveObstacleWeight(DV2O, obstacle.safety_dist);
            
    //         if (qV2O > 0) {
    //             double eV2O = DV2O - obstacle.safety_dist;
    //             cost += qV2O * eV2O * eV2O; 
    //         }
    //       }
          
    //       // Input rate costs
    //       Eigen::Vector3d du(U(i,0)-prev_v, U(i,1)-prev_ω, U(i,2)-prev_vs);
    //       cost += du.transpose()*R_*du;
    //       prev_v = U(i,0); prev_ω = U(i,1); prev_vs= U(i,2);
    //     }
    //     return cost;
    //   });


    // controller_.setObjectiveFunction(
    // [this](const mpc::mat<Tph + 1, Tnx>& X, const mpc::mat<Tph + 1, Tny>&, const mpc::mat<Tph + 1, Tnu>& U, const double&){
    //   double cost = 0, prev_v=0, prev_ω=0, prev_vs=0;
      
    //   for (int i = 0; i <= Tph; ++i) {
    //     // Standard MPCC tracking costs
    //     double s = X(i,3);
    //     auto ref = spline_.getPoint(s);
    //     auto d   = spline_.getDerivatives(s);
    //     double θ = std::atan2(d.dy_ds, d.dx_ds);
    //     double e_c =  std::sin(θ)*(X(i,0)-ref.x) - std::cos(θ)*(X(i,1)-ref.y);
    //     double e_l = -std::cos(θ)*(X(i,0)-ref.x) - std::sin(θ)*(X(i,1)-ref.y);
    //     Eigen::Vector2d err(e_c,e_l);
    //     cost += err.transpose()*Q_*err;
    //     cost += -qθ_*s;
      
    //     for (const auto& obstacle : obstacles_) {
        
    //     auto obstacle_pos = spline_.getPoint(obstacle.s);
        
    //     double DV2O = computeV2ODistance(X(i,0), X(i,1), 
    //                                     obstacle_pos.x, obstacle_pos.y, 
    //                                     obstacle.radius);
    //       double qV2O = getAdaptiveObstacleWeight(DV2O, obstacle.safety_dist);
          
    //       if (qV2O > 0) {
    //           double eV2O = DV2O - obstacle.safety_dist;
    //           cost += qV2O * eV2O * eV2O; 
    //       }
    //     }
        
    //     // Input rate costs
    //     Eigen::Vector3d du(U(i,0)-prev_v, U(i,1)-prev_ω, U(i,2)-prev_vs);
    //     cost += du.transpose()*R_*du;
    //     prev_v = U(i,0); prev_ω = U(i,1); prev_vs= U(i,2);
    //   }
    //   return cost;
    // });

    controller_.setObjectiveFunction(
  [this](const mpc::mat<Tph + 1, Tnx>& X, const mpc::mat<Tph + 1, Tny>&, const mpc::mat<Tph + 1, Tnu>& U, const double&){
    double cost = 0, prev_v=0, prev_ω=0, prev_vs=0;
    
    for (int i = 0; i <= Tph; ++i) {
      // Standard MPCC tracking costs
      double s = X(i,3);
      auto ref = spline_.getPoint(s);
      auto d   = spline_.getDerivatives(s);
      double θ = std::atan2(d.dy_ds, d.dx_ds);
      double e_c =  std::sin(θ)*(X(i,0)-ref.x) - std::cos(θ)*(X(i,1)-ref.y);
      double e_l = -std::cos(θ)*(X(i,0)-ref.x) - std::sin(θ)*(X(i,1)-ref.y);
      Eigen::Vector2d err(e_c,e_l);
      cost += err.transpose()*Q_*err;
      cost += -qθ_*s;
      
      // ✅ MODIFIED: Use predictive positions for dynamic obstacles
      for (const auto& obstacle : obstacles_) {
        std::pair<double, double> obs_pos;
        
        if (std::abs(obstacle.vs) < 1e-6) {
          // Static obstacle - use current position
          obs_pos = obstacle.getCurrentPosition(spline_);
        } else {
          // ✅ Dynamic obstacle - predict future position
          obs_pos = obstacle.predictPosition(dt_, i, spline_);
        }
        
        double DV2O = computeV2ODistance(X(i,0), X(i,1), 
                                        obs_pos.first, obs_pos.second,
                                        obstacle.radius);
        double qV2O = getAdaptiveObstacleWeight(DV2O, obstacle.safety_dist);
        
        if (qV2O > 0) {
          double eV2O = DV2O - obstacle.safety_dist;
          cost += qV2O * eV2O * eV2O; 
        }
      }
      
      // Input rate costs
      Eigen::Vector3d du(U(i,0)-prev_v, U(i,1)-prev_ω, U(i,2)-prev_vs);
      cost += du.transpose()*R_*du;
      prev_v = U(i,0); prev_ω = U(i,1); prev_vs= U(i,2);
    }
    return cost;
  });


    mpc::NLParameters p;
    p.maximum_iteration = iterations;
    p.relative_ftol     = 1e-2;
    p.relative_xtol     = 1e-2;
    p.absolute_ftol   = 1e-2;
    p.absolute_xtol   = 1e-2;
    p.hard_constraints  = false;
    p.enable_warm_start = true;
    controller_.setOptimizerParameters(p);
  }

  void setupObstacles() {
    obstacles_.clear();
    
    // Add obstacle following same trajectory at slower speed
    Obstacle obs1;
    obs1.s = 2.0;                    // Start 2 meters ahead on trajectory
    obs1.vs = 0.01;                  // Move at 50% of max vehicle speed
    obs1.radius = 0.1;
    obs1.safety_dist = 0.5;
    obs1.id = 0;
    obs1.confidence = 1.0;
    obstacles_.push_back(obs1);
    
    RCLCPP_INFO(get_logger(), "Initialized %zu trajectory-following obstacles", obstacles_.size());
}


  void publishTrackBoundaries() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Clear any existing markers first
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = now();
    clear_marker.ns = "track_boundaries";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    // Create left and right boundary lines
    visualization_msgs::msg::Marker left_boundary, right_boundary;
    
    // Left boundary setup
    left_boundary.header.frame_id = "map";
    left_boundary.header.stamp = now();
    left_boundary.ns = "track_boundaries";
    left_boundary.id = 0;
    left_boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
    left_boundary.action = visualization_msgs::msg::Marker::ADD;
    left_boundary.scale.x = 0.05; // Line width
    left_boundary.color.r = 1.0;  // Red for left boundary
    left_boundary.color.g = 0.0;
    left_boundary.color.b = 0.0;
    left_boundary.color.a = 0.8;
    
    // Right boundary setup
    right_boundary.header.frame_id = "map";
    right_boundary.header.stamp = now();
    right_boundary.ns = "track_boundaries";
    right_boundary.id = 1;
    right_boundary.type = visualization_msgs::msg::Marker::LINE_STRIP;
    right_boundary.action = visualization_msgs::msg::Marker::ADD;
    right_boundary.scale.x = 0.05; // Line width
    right_boundary.color.r = 0.0;
    right_boundary.color.g = 1.0;  // Green for right boundary
    right_boundary.color.b = 0.0;
    right_boundary.color.a = 0.8;
    
    // Generate boundary points
    const auto& trajectory_points = spline_.points();
    for (size_t i = 0; i < trajectory_points.size(); ++i) {
        const auto& pt = trajectory_points[i];
        
        // Calculate direction perpendicular to path
        geometry_msgs::msg::Point center_point;
        center_point.x = pt.x;
        center_point.y = pt.y;
        center_point.z = 0.0;
        
        // Get path direction for normal calculation
        double dx = 0, dy = 0;
        if (i < trajectory_points.size() - 1) {
            dx = trajectory_points[i+1].x - pt.x;
            dy = trajectory_points[i+1].y - pt.y;
        } else if (i > 0) {
            dx = pt.x - trajectory_points[i-1].x;
            dy = pt.y - trajectory_points[i-1].y;
        }
        
        // Normalize direction
        double length = std::sqrt(dx*dx + dy*dy);
        if (length > 1e-6) {
            dx /= length;
            dy /= length;
        }
        
        // Calculate normal vector (perpendicular to path)
        double normal_x = -dy; // Left normal
        double normal_y = dx;
        
        // Left boundary point
        geometry_msgs::msg::Point left_point;
        left_point.x = pt.x + normal_x * pt.left_bound;
        left_point.y = pt.y + normal_y * pt.left_bound;
        left_point.z = 0.0;
        left_boundary.points.push_back(left_point);
        
        // Right boundary point  
        geometry_msgs::msg::Point right_point;
        right_point.x = pt.x - normal_x * pt.right_bound; // Right normal
        right_point.y = pt.y - normal_y * pt.right_bound;
        right_point.z = 0.0;
        right_boundary.points.push_back(right_point);
    }
    
    marker_array.markers.push_back(left_boundary);
    marker_array.markers.push_back(right_boundary);
    
    // Optional: Add boundary fill area
    visualization_msgs::msg::Marker boundary_area;
    boundary_area.header.frame_id = "map";
    boundary_area.header.stamp = now();
    boundary_area.ns = "track_boundaries";
    boundary_area.id = 2;
    boundary_area.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    boundary_area.action = visualization_msgs::msg::Marker::ADD;
    boundary_area.scale.x = 1.0;
    boundary_area.scale.y = 1.0;
    boundary_area.scale.z = 1.0;
    boundary_area.color.r = 0.5;
    boundary_area.color.g = 0.5;
    boundary_area.color.b = 0.5;
    boundary_area.color.a = 0.2; // Very transparent
    
    // Create triangular strips between boundaries
    for (size_t i = 0; i < left_boundary.points.size() - 1; ++i) {
        // Triangle 1
        boundary_area.points.push_back(left_boundary.points[i]);
        boundary_area.points.push_back(right_boundary.points[i]);
        boundary_area.points.push_back(left_boundary.points[i+1]);
        
        // Triangle 2  
        boundary_area.points.push_back(right_boundary.points[i]);
        boundary_area.points.push_back(right_boundary.points[i+1]);
        boundary_area.points.push_back(left_boundary.points[i+1]);
    }
    
    marker_array.markers.push_back(boundary_area);
    boundary_pub_->publish(marker_array);
}


  void initState() {
    x0_.setZero();
    u0_.setZero();
    
    // Vehicle initial state only: [x, y, psi, s]
    x0_ << 0.0, 0.0, 0.0, 0.0;
    u0_ << 0.0, 0.0, 0.2*v_s_max_;

    nav_msgs::msg::Odometry cmd;
    cmd.header.frame_id = "base_link";
    cmd.header.stamp = now();
    cmd.twist.twist.linear.x  = u0_(0);
    cmd.twist.twist.angular.z = u0_(1);
    cmd.twist.twist.linear.y  = u0_(2);
    cmd_pub_->publish(cmd);
  }

  double computeV2ODistance(double vehicle_x, double vehicle_y, double obstacle_x, double obstacle_y, double obstacle_radius) const {
      double dx = vehicle_x - obstacle_x;
      double dy = vehicle_y - obstacle_y;
      double distance = std::sqrt(dx*dx + dy*dy);
      return distance - obstacle_radius - vehicle_radius_;
  }

  double getAdaptiveObstacleWeight(double DV2O, double DSft_O) const {
    if (DV2O < 0) {
        return Pk;  
    } else if (DV2O <= DSft_O && DV2O >= 0) {
        return Pk * std::exp((-2 * DV2O * DV2O)/(DSft_O*DSft_O));
    } else{
        return 0.0; 
    }
  }

void publishObstacles() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Publish obstacles and their safety zones
    for (size_t i = 0; i < obstacles_.size(); ++i) {
      const auto& obstacle = obstacles_[i];

      auto pos = obstacle.getCurrentPosition(spline_);
      double obs_x = pos.first;
      double obs_y = pos.second;

      
      // 1. Obstacle physical marker (red cylinder)
      visualization_msgs::msg::Marker obstacle_marker;
      obstacle_marker.header.frame_id = "map";
      obstacle_marker.header.stamp = now();
      obstacle_marker.ns = "obstacles_physical";
      obstacle_marker.id = static_cast<int>(i);
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
      obstacle_marker.color.g = 0.0;
      obstacle_marker.color.b = 0.0;
      obstacle_marker.color.a = 0.9;
      
      marker_array.markers.push_back(obstacle_marker);
      
      // 2. Safety zone marker (transparent orange/yellow cylinder)
      visualization_msgs::msg::Marker safety_zone_marker;
      safety_zone_marker.header.frame_id = "map";
      safety_zone_marker.header.stamp = now();
      safety_zone_marker.ns = "obstacle_safety_zone";
      safety_zone_marker.id = static_cast<int>(i) + 1000;  // Use high IDs to avoid conflicts
      safety_zone_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      safety_zone_marker.action = visualization_msgs::msg::Marker::ADD;
      
      safety_zone_marker.pose.position.x = obs_x;
      safety_zone_marker.pose.position.y = obs_y;
      safety_zone_marker.pose.position.z = 0.1;  // Slightly lower than obstacle
      safety_zone_marker.pose.orientation.w = 1.0;
      
      // Safety zone radius = obstacle radius + safety distance
      double safety_zone_radius = obstacle.radius + obstacle.safety_dist;
      safety_zone_marker.scale.x = safety_zone_radius * 2.0;
      safety_zone_marker.scale.y = safety_zone_radius * 2.0;
      safety_zone_marker.scale.z = 0.05;  // Thinner cylinder
      
      safety_zone_marker.color.r = 1.0;   // Orange/yellow color
      safety_zone_marker.color.g = 0.6;
      safety_zone_marker.color.b = 0.0;
      safety_zone_marker.color.a = 0.3;   // Transparent
      
      marker_array.markers.push_back(safety_zone_marker);
    }
    
    // Vehicle marker (unchanged)
    visualization_msgs::msg::Marker vehicle_marker;
    vehicle_marker.header.frame_id = "map";
    vehicle_marker.header.stamp = now();
    vehicle_marker.ns = "vehicle_physical";
    vehicle_marker.id = 100; // Use high ID to avoid conflict
    vehicle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
    
    vehicle_marker.pose.position.x = x0_(0);
    vehicle_marker.pose.position.y = x0_(1);
    vehicle_marker.pose.position.z = 0.0;
    vehicle_marker.pose.orientation.w = 1.0;
    
    vehicle_marker.scale.x = vehicle_radius_ * 2.0;
    vehicle_marker.scale.y = vehicle_radius_ * 2.0;
    vehicle_marker.scale.z = 0.05;
    
    vehicle_marker.color.r = 0.0;
    vehicle_marker.color.g = 0.0;
    vehicle_marker.color.b = 0.8;
    vehicle_marker.color.a = 0.9;
    
    marker_array.markers.push_back(vehicle_marker);
    obstacle_pub_->publish(marker_array);
}

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update vehicle states 
    x0_(0) = msg->pose.pose.position.x;
    x0_(1) = msg->pose.pose.position.y;
    double qw = msg->pose.pose.orientation.w;
    double qz = msg->pose.pose.orientation.z;
    x0_(2) = 2.0 * std::atan2(qz, qw);

    // Update obstacle states
    for (auto& obstacle : obstacles_) {
      obstacle.update(dt_);
    }

    //pubish obstacle data to terminal:
    RCLCPP_INFO(get_logger(), "Vehicle state: x=%.2f, y=%.2f, psi=%.2f, s=%.2f", 
                x0_(0), x0_(1), x0_(2), x0_(3));
    for (const auto& obstacle : obstacles_) {
      auto pos = obstacle.getCurrentPosition(spline_);
      RCLCPP_INFO(get_logger(), "Obstacle %d: s=%.2f, vs=%.2f, radius=%.2f, safety_dist=%.2f, position=(%.2f, %.2f)",
                  obstacle.id, obstacle.s, obstacle.vs,
                  obstacle.radius, obstacle.safety_dist,
                  pos.first, pos.second);
    } 

    
    // Publish actual path
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = now();
    ps.pose.position.x = x0_(0);
    ps.pose.position.y = x0_(1);
    ps.pose.orientation.z = x0_(2);
    ps.pose.orientation.w = 1.0;
    actual_path_.push_back(ps);
    nav_msgs::msg::Path act;
    act.header = ps.header;
    act.poses = actual_path_;
    actual_pub_->publish(act);

    // Run MPCC optimization
    auto res = controller_.optimize(x0_, u0_);
    // auto seq = controller_.getOptimalSequence();

    x0_(3) = x0_(3) + res.cmd(2)*dt_;  // Update s

    publishObstacles();
    
    // Send commands
    nav_msgs::msg::Odometry cmd;
    cmd.header.frame_id = "base_link";
    cmd.header.stamp = now();
    cmd.twist.twist.linear.x  = res.cmd(0);
    cmd.twist.twist.angular.z = res.cmd(1);
    cmd.twist.twist.linear.y  = res.cmd(2);
    cmd.twist.twist.linear.z  = dt_;
    cmd.twist.twist.angular.x = x0_(3);

    auto ref = spline_.getPoint(x0_(3));
    geometry_msgs::msg::PointStamped ref_point;
    ref_point.header.frame_id = "map";
    ref_point.header.stamp = now();
    ref_point.point.x = ref.x;
    ref_point.point.y = ref.y;
    ref_point.point.z = 0.0;
    reference_point_pub_->publish(ref_point);

    geometry_msgs::msg::Twist tv;
    tv.linear.x = res.cmd(0);
    tv.angular.z = res.cmd(1);
    vel_pub_->publish(tv);
    cmd_pub_->publish(cmd);
    u0_ = res.cmd;
  }

  // Members
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_, actual_pub_, predicted_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_; 
  std::vector<geometry_msgs::msg::PoseStamped> actual_path_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr reference_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundary_pub_;

  Eigen::Vector<double, Tnx> x0_;
  Eigen::Vector<double, Tnu> u0_;

  LinearSpline spline_;
  double v_max_, ω_max_, v_s_max_, rate_, dt_;
  
  // Vehicle parameters
  double vehicle_radius_ = 0.1;
  
  // Obstacle collection
  std::vector<Obstacle> obstacles_;
 
  // Cost parameters
  const double k_v_ = 0.1, k_ω_ =0.1, k_vs_ = 0.1;
const double k_c_ = 50.0, k_l_ = 100.0, k_th_ = 3000.0;
const double Pk = 150.0;

  const double qθ_ = k_th_/100.0;
  const Eigen::Matrix2d Q_ = (Eigen::Matrix2d() << k_c_/pow(0.5,2), 0, 0, k_l_/pow(0.5,2)).finished();
  const Eigen::Matrix3d R_ = (Eigen::Matrix3d() << 
     k_v_/(std::pow(v_max_, 2)), 0, 0,
     0, k_ω_/std::pow(ω_max_,2), 0,
     0, 0, k_vs_/(pow(v_s_max_, 2))).finished();
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCCController>());
  rclcpp::shutdown();
  return 0;
}
