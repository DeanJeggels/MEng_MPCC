#include "/home/deanjeggs/new_ros2_ws/src/mpcc/include/mpcc/linear_spline.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

namespace voyager_mpcc {

LinearSpline::LinearSpline() {}

bool LinearSpline::loadFromCSV(const std::string& filename) {
    trajectory_.clear();
    s_values_.clear();
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        TrajectoryPoint point;
        
        // Read x coordinate
        if (std::getline(ss, token, ',')) {
            point.x = std::stod(token);
        } else {
            continue; 
        }
        
        // Read y coordinate
        if (std::getline(ss, token, ',')) {
            point.y = std::stod(token);
        } else {
            continue; 
        }
        
        trajectory_.push_back(point);
    }
    
    // Calculate progress parameter
    calculateProgressParameter();
    
    return !trajectory_.empty();
}

void LinearSpline::calculateProgressParameter() {
    s_values_.resize(trajectory_.size());
    s_values_[0] = 0.0;  // Start at zero
    
    for (size_t i = 1; i < trajectory_.size(); i++) {
        double dx = trajectory_[i].x - trajectory_[i-1].x;
        double dy = trajectory_[i].y - trajectory_[i-1].y;
        double segment_length = std::sqrt(dx*dx + dy*dy);
        s_values_[i] = s_values_[i-1] + segment_length;
    }
}

size_t LinearSpline::size() const {
    return trajectory_.size();
}

TrajectoryPoint LinearSpline::getPointAtProgress(double s) const {
    
    if (trajectory_.empty()) {
        return {0.0, 0.0};
    }
    
    if (s <= s_values_.front()) {
        return trajectory_.front();
    }
    
    if (s >= s_values_.back()) {
        return trajectory_.back();
    }
    
    // Find the segment containing s
    auto it = std::lower_bound(s_values_.begin(), s_values_.end(), s);
    size_t i = it - s_values_.begin();
    
    if (i > 0 && s < s_values_[i]) {
        i--; // Adjust to get the segment before the progress value
    }
    
    // Linear interpolation within the segment
    double s0 = s_values_[i];
    double s1 = s_values_[i+1];
    double t = (s - s0) / (s1 - s0);  
    TrajectoryPoint result;
    result.x = trajectory_[i].x + t * (trajectory_[i+1].x - trajectory_[i].x);
    result.y = trajectory_[i].y + t * (trajectory_[i+1].y - trajectory_[i].y);
    
    return result;
}

PathDerivatives LinearSpline::getDerivativesAtProgress(double s) const {
    
    if (trajectory_.size() < 2) {
        return {0.0, 0.0};
    }
    

    auto it = std::lower_bound(s_values_.begin(), s_values_.end(), s);
    size_t i = it - s_values_.begin();
    
    if (i > 0 && s < s_values_[i]) {
        i--; 
    }
    
    if (i >= trajectory_.size() - 1) {
        i = trajectory_.size() - 2; 
    }

    double segment_length = s_values_[i+1] - s_values_[i];
    
    if (segment_length < 1e-6) {
  
        if (i + 2 < trajectory_.size()) {
            i++;
            segment_length = s_values_[i+1] - s_values_[i];
        }
    }
    
    PathDerivatives result;
    
    if (segment_length < 1e-6) {
        result.dx_ds = 1.0;
        result.dy_ds = 0.0;
    } else {
        result.dx_ds = (trajectory_[i+1].x - trajectory_[i].x) / segment_length;
        result.dy_ds = (trajectory_[i+1].y - trajectory_[i].y) / segment_length;
    }
    
    return result;
}

double LinearSpline::getMaxProgress() const {
    return s_values_.empty() ? 0.0 : s_values_.back();
}

const std::vector<TrajectoryPoint>& LinearSpline::getTrajectoryPoints() const {
    return trajectory_;
}

const std::vector<double>& LinearSpline::getProgressValues() const {
    return s_values_;
}

} // namespace voyager_mpcc