#ifndef VOYAGER_MPCC_OPTIMIZATION_PROBLEM_HPP
#define VOYAGER_MPCC_OPTIMIZATION_PROBLEM_HPP

#include <vector>
#include "mpcc/linear_spline.hpp"

namespace voyager_mpcc {

struct TrackingErrors {
    double contouring_error;
    double lag_error;
};

struct MPCCSolution {
    std::vector<double> v;      // Linear velocity
    std::vector<double> omega;  // Angular velocity
    std::vector<double> v_s;    // Progress velocity
};

struct MPCCParameters {
    double q_contour;    // Weight for contouring error
    double q_lag;        // Weight for lag error
    double q_progress;   // Weight for maximizing progress
    double q_v;          // Weight for linear velocity regulation
    double q_omega;      // Weight for angular velocity regulation
    double q_v_s;        // Weight for progress velocity regulation
    
    double v_ref;        // Reference linear velocity
    double v_max;        // Maximum linear velocity
    double omega_max;    // Maximum angular velocity
    double v_s_max;      // Maximum progress velocity
    
    double dt;           // Time step
    int horizon;         // Prediction horizon
};

class OptimizationProblem {
public:
    // Initialize the optimization problem with parameters
    OptimizationProblem(const MPCCParameters& params);
    
    // Calculate tracking errors (contouring and lag)
    TrackingErrors calculateErrors(
        double robot_x, double robot_y, double s,
        const LinearSpline& spline) const;
    
    // Solve the MPCC optimization problem
    MPCCSolution solve(
        double current_x, double current_y, double current_psi, double current_s,
        const LinearSpline& spline);
    
    // Normalize angle to [-pi, pi]
    static double normalizeAngle(double angle);

private:
    MPCCParameters params_;
};

} // namespace voyager_mpcc

#endif // VOYAGER_MPCC_OPTIMIZATION_PROBLEM_HPP