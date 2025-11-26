// mpcc_controller.cpp

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

constexpr int Tnx   = 4;   // [x, y, psi, s]
constexpr int Tnu   = 3;   // [v, ω, v_s]
constexpr int Tny   = 4;  

constexpr int Tph   = 3;  
constexpr int Tch   = 3;  
constexpr int iterations = 1000;


constexpr int Tineq = 6*(Tph+1);
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
    
    double getTotalLength() const {
        return s_.empty() ? 0.0 : s_.back();
    }

private:
    std::vector<TrajectoryPoint> trajectory_;
    std::vector<double> s_;
};

struct Obstacle {
    double s;
    double vs;
    double x, y;       
    double radius;     
    double safety_dist;
    
    std::pair<double, double> getCurrentPosition(const LinearSpline& spline) const {
        auto current_point = spline.getPoint(s);
        return {current_point.x, current_point.y + 0.1};  
    }
    
    std::pair<double, double> predictPosition(double dt, int steps, const LinearSpline& spline) const {
        double future_s = s + vs * dt * steps;
        auto future_point = spline.getPoint(future_s);
        return {future_point.x, future_point.y + 0.1};  
    }
    
    void update(double dt, double track_length = 0.0) {
        s += vs * dt;
        
        if (track_length > 0.0 && s >= track_length) {
            s -= track_length;
        }
    }
};

class MPCCController : public rclcpp::Node {
public:
    MPCCController()
    : Node("mpcc_controller"),
      v_max_(0.05), ω_max_(M_PI/2), v_s_max_(0.05), rate_(5.0), dt_(1.0/rate_)
    {
        if (!spline_.loadFromCSV("/home/deanjeggs/new_ros2_ws/src/mpcc/racetracks/recorded_path_straight_offset.csv")) {
            RCLCPP_ERROR(get_logger(), "Failed loading spline");
            rclcpp::shutdown();
            return;
        }


        track_length_ = spline_.getTotalLength();
        lap_start_time_ = this->now().seconds();
        
        RCLCPP_INFO(get_logger(), "Track length: %.2f meters", track_length_);
        RCLCPP_INFO(get_logger(), "Multi-lap mode: %d laps target", desired_laps_);

        using std::placeholders::_1;
        localization_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization_pose", 10,
            std::bind(&MPCCController::localizationPoseCallback, this, std::placeholders::_1));
        predicted_pub_ = create_publisher<nav_msgs::msg::Path>("/mpcc_obst/predicted_path", 10);
        vel_pub_       = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        cmd_pub_ = create_publisher<nav_msgs::msg::Odometry>("/cmd_odom", 10);
        actual_pub_    = create_publisher<nav_msgs::msg::Path>("/mpcc_obst/actual_path", 10);
        reference_pub_ = create_publisher<nav_msgs::msg::Path>("/mpcc_obst/reference_path", 10);
        reference_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/mpcc_obst/current_reference_point", 10);

        publishReference();
        setupController();
        initState();
    }

private:

    double track_length_ = 0.0;         
    int laps_completed_ = 0;            
    int desired_laps_ = 10;                
    double lap_start_time_ = 0.0;         
    std::vector<double> lap_times_;       
    bool multi_lap_mode_ = true;          
    
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
        RCLCPP_INFO(get_logger(), "Total distance: %.2f meters", desired_laps_ * track_length_);
    }


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
        [this](mpc::cvec<Tineq>& ineq, const mpc::mat<Tph + 1, Tnx>& x,  const mpc::mat<Tph + 1, Tny>&,  const mpc::mat<Tph + 1, Tnu>& u, const double&) {
            int idx = 0;
            
            // SINGLE LOOP: Input constraints, track boundaries, AND obstacle constraints
            for (int i = 0; i <= Tph; ++i) {
                // Input constraints
                ineq(idx++) = u(i,0) - v_max_;
                ineq(idx++) = -u(i,0) + 0.2*v_max_;
                ineq(idx++) = u(i,1) - ω_max_;
                ineq(idx++) = -u(i,1) - ω_max_;
                ineq(idx++) = u(i,2) - v_s_max_;
                ineq(idx++) = -u(i,2) + 0.2*v_s_max_;
                
            }
        }, 1e-6);

        controller_.setStateSpaceFunction(
            [this](mpc::cvec<Tnx>& x_next, const mpc::cvec<Tnx>& x,  const mpc::cvec<Tnu>& u, const unsigned int&){
                double dt = 1.0/rate_;
                x_next(0) = x(0) + u(0)*std::cos(x(2))*dt;
                x_next(1) = x(1) + u(0)*std::sin(x(2))*dt;
                x_next(2) = x(2) + u(1)*dt;
                x_next(3) = x(3) + u(2)*dt;
            });

        controller_.setOutputFunction(
            [](auto &y, const auto &x, const auto&, unsigned){ y = x; });

        controller_.setObjectiveFunction(
            [this](const mpc::mat<Tph + 1, Tnx>& X, const mpc::mat<Tph + 1, Tny>&, const mpc::mat<Tph + 1, Tnu>& U, const double&){
                double cost = 0, prev_v=0, prev_ω=0, prev_vs=0;
                
                for (int i = 0; i <= Tph; ++i) {
                    double s = X(i,3);
                    auto ref = spline_.getPoint(s);
                    auto d   = spline_.getDerivatives(s);
                    double θ = std::atan2(d.dy_ds, d.dx_ds);
                    double e_c =  std::sin(θ)*(X(i,0)-ref.x) - std::cos(θ)*(X(i,1)-ref.y);
                    double e_l = -std::cos(θ)*(X(i,0)-ref.x) - std::sin(θ)*(X(i,1)-ref.y);
                    Eigen::Vector2d err(e_c,e_l);
                    cost += err.transpose()*Q_*err;
                    cost += -qθ_*s;
                    Eigen::Vector3d du(U(i,0)-prev_v, U(i,1)-prev_ω, U(i,2)-prev_vs);
                    cost += du.transpose()*R_*du;
                    prev_v = U(i,0); prev_ω = U(i,1); prev_vs= U(i,2);
                }
                return cost;
            });

        mpc::NLParameters p;
        p.maximum_iteration = iterations;
        p.relative_ftol     = 1e-8;
        p.relative_xtol     = 1e-8;
        p.absolute_ftol   = 1e-8;
        p.absolute_xtol   = 1e-8;
        p.hard_constraints  = false;
        p.enable_warm_start = true;
        controller_.setOptimizerParameters(p);
    }

    void initState() {
        x0_.setZero();
        u0_.setZero();
        x0_ << 0,0.45,0,0;
        u0_ << 0.0,0,0.2*v_s_max_;

        nav_msgs::msg::Odometry cmd;
        cmd.header.frame_id = "base_link";
        cmd.header.stamp = now();
        cmd.twist.twist.linear.x  = u0_(0);
        cmd.twist.twist.angular.z = u0_(1);
        cmd.twist.twist.linear.y  = u0_(2);
        cmd_pub_->publish(cmd);
    }


    void localizationPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        auto start = std::chrono::steady_clock::now();

        x0_(0) = msg->pose.pose.position.x;
        x0_(1) = msg->pose.pose.position.y;
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        x0_(2) = 2.0 * std::atan2(qz, qw);

        auto res = controller_.optimize(x0_, u0_); 

        // Get optimal sequence and publish predicted path
        auto sequence = controller_.getOptimalSequence();
        
        // Create predicted path message
        nav_msgs::msg::Path predicted_path;
        predicted_path.header.frame_id = "map";
        predicted_path.header.stamp = now();
        
        // Extract predicted states (i=1 to Tph, j=0,1 for x,y)
        for (int i = 1; i <= Tph; ++i) {  // Start from i=1 (skip current state)
            geometry_msgs::msg::PoseStamped predicted_pose;
            predicted_pose.header = predicted_path.header;
            
            // Extract x and y from optimal states
            predicted_pose.pose.position.x = sequence.state(i, 0);  // j=0 for x
            predicted_pose.pose.position.y = sequence.state(i, 1);  // j=1 for y
            predicted_pose.pose.position.z = 0.0;
            
            // Optional: Set orientation from heading (psi = sequence.state(i, 2))
            double psi = sequence.state(i, 2);
            predicted_pose.pose.orientation.z = std::sin(psi/2.0);
            predicted_pose.pose.orientation.w = std::cos(psi/2.0);
            
            predicted_path.poses.push_back(predicted_pose);
        }
        
        // Publish the predicted path
        predicted_pub_->publish(predicted_path);

        double old_s = x0_(3);
        x0_(3) = x0_(3) + res.cmd(2) * dt_;
        

        if (multi_lap_mode_ && x0_(3) >= track_length_) {
            laps_completed_++;
            x0_(3) -= track_length_;

            double current_time = this->now().seconds();
            double lap_time = current_time - lap_start_time_;
            lap_times_.push_back(lap_time);
            
            RCLCPP_INFO(get_logger(), "🏁 Completed Lap %d/%d in %.2f seconds", 
                        laps_completed_, desired_laps_, lap_time);
            
            lap_start_time_ = current_time;
            
            if (laps_completed_ >= desired_laps_) {
                RCLCPP_INFO(get_logger(), "🎉 ALL %d LAPS COMPLETED! 🎉", desired_laps_);
                printLapSummary();
                
                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.angular.z = 0.0;
                vel_pub_->publish(stop_cmd);
                
                rclcpp::shutdown();
                return;
            }
        }


        auto ref = spline_.getPoint(x0_(3));
        geometry_msgs::msg::PointStamped ref_point;
        ref_point.header.frame_id = "map";
        ref_point.header.stamp = now();
        ref_point.point.x = ref.x;
        ref_point.point.y = ref.y;
        ref_point.point.z = 0.0;
        reference_point_pub_->publish(ref_point);

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

        
        nav_msgs::msg::Odometry cmd;
        cmd.header.frame_id = "base_link";
        cmd.header.stamp = now();
        cmd.twist.twist.linear.x  = res.cmd(0);
        cmd.twist.twist.angular.z = res.cmd(1);
        cmd.twist.twist.linear.y  = res.cmd(2);
        cmd.twist.twist.linear.z  = dt_;
        cmd.twist.twist.angular.x = x0_(3);

        geometry_msgs::msg::Twist tv;
        tv.linear.x = res.cmd(0);
        tv.angular.z= res.cmd(1);

        cmd_pub_->publish(cmd);
        vel_pub_->publish(tv);
        u0_ = res.cmd;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_, actual_pub_, predicted_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr states_pub_, inputs_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr opt_time_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr reference_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundary_pub_;
    std::vector<geometry_msgs::msg::PoseStamped> actual_path_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_pose_sub_;


    Eigen::Vector<double, Tnx> x0_;
    Eigen::Vector<double, Tnu> u0_;

    LinearSpline spline_;
    double v_max_, ω_max_, v_s_max_, rate_, dt_;

    const double k_v_=0.1, k_ω_=0.1, k_vs_=0.1;
    const double k_c_ = 100.0, k_l_ = 100.0, k_th_ = 4000.0;


    const double qθ_ = k_th_/100.0;
    const Eigen::Matrix2d Q_ = (Eigen::Matrix2d() << k_c_/pow(0.5,2), 0, 0, k_l_/pow(0.5,2)).finished();
    const Eigen::Matrix3d R_ = (Eigen::Matrix3d() << 
       k_v_/(std::pow(v_max_, 2)), 0,       0,
       0,        k_ω_/std::pow(ω_max_,2), 0,
       0,        0,       k_vs_/(pow(v_s_max_, 2))).finished();
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCCController>());
    rclcpp::shutdown();
    return 0;
}
