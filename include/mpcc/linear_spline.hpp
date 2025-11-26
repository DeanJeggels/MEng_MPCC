#ifndef MPCC_LINEAR_SPLINE_HPP
#define MPCC_LINEAR_SPLINE_HPP

#include <vector>
#include <string>

namespace voyager_mpcc {

struct TrajectoryPoint {
    double x;
    double y;
};

struct PathDerivatives {
    double dx_ds;  // dx/ds - tangent x component
    double dy_ds;  // dy/ds - tangent y component
};

class LinearSpline {
public:
    // Constructor - initializes an empty spline
    LinearSpline();
    
    // Load trajectory from CSV file
    bool loadFromCSV(const std::string& filename);
    
    // Get the number of points in the trajectory
    size_t size() const;
    
    // Get interpolated point at a specific progress value
    TrajectoryPoint getPointAtProgress(double s) const;
    
    // Get derivatives at a specific progress value
    PathDerivatives getDerivativesAtProgress(double s) const;
    
    // Get the maximum progress value (total path length)
    double getMaxProgress() const;
    
    // Get the raw trajectory points
    const std::vector<TrajectoryPoint>& getTrajectoryPoints() const;
    
    // Get the raw progress values
    const std::vector<double>& getProgressValues() const;

private:
    std::vector<TrajectoryPoint> trajectory_;
    std::vector<double> s_values_;  // Progress parameter for each point
    
    // Calculate the progress parameter for the loaded trajectory
    void calculateProgressParameter();
};

} // namespace voyager_mpcc

#endif // VOYAGER_MPCC_LINEAR_SPLINE_HPP