#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <fstream>
#include <iomanip>

class OccupancyExtractor : public rclcpp::Node
{
public:
    OccupancyExtractor() 
    : Node("occupancy_extractor"), occupancy_threshold_(50), frame_id_("map")
    {
        // Declare parameters
        this->declare_parameter("occupancy_threshold", occupancy_threshold_);
        this->declare_parameter("frame_id", frame_id_);
        this->declare_parameter("output_csv_path", std::string("/home/deanjeggs/new_ros2_ws/src/mpcc/obstacles/occupied_cells.csv"));
        
        // Get parameters
        occupancy_threshold_ = this->get_parameter("occupancy_threshold").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        output_csv_path_ = this->get_parameter("output_csv_path").as_string();

        // Set up QoS profile for map subscription
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();

        // Create subscription with proper QoS
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos, 
            std::bind(&OccupancyExtractor::mapCallback, this, std::placeholders::_1));

        // Create publishers for visualization
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/occupied_cells/markers", 10);

        RCLCPP_INFO(this->get_logger(), "OccupancyExtractor node started");
        RCLCPP_INFO(this->get_logger(), "Occupancy threshold: %d", occupancy_threshold_);
        RCLCPP_INFO(this->get_logger(), "Output CSV path: %s", output_csv_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waiting for map data on /map topic...");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Map received! Processing occupancy grid: %dx%d, resolution: %.3f", 
            msg->info.width, msg->info.height, msg->info.resolution);

        // Extract occupied cells
        auto occupied_points = extractOccupiedCells(msg);
        
        RCLCPP_INFO(this->get_logger(), "Found %zu occupied cells", occupied_points.size());

        // Save to CSV
        if (!occupied_points.empty()) {
            saveOccupiedCellsToCSV(occupied_points);
            publishVisualizationMarkers(occupied_points);
            RCLCPP_INFO(this->get_logger(), "Saved occupied cells to CSV and published visualization");
        } else {
            RCLCPP_WARN(this->get_logger(), "No occupied cells found with threshold %d", occupancy_threshold_);
        }

        // Shutdown after processing one map (optional)
        RCLCPP_INFO(this->get_logger(), "Processing complete. Shutting down node.");
        rclcpp::shutdown();
    }

    std::vector<geometry_msgs::msg::Point> extractOccupiedCells(
    const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
{
    std::vector<geometry_msgs::msg::Point> boundary_points;
    
    const auto& info = grid->info;
    const auto& data = grid->data;
    
    // 8-connected neighborhood offsets
    std::vector<std::pair<int, int>> neighbors = {
        {-1, -1}, {-1, 0}, {-1, 1},
        { 0, -1},          { 0, 1},
        { 1, -1}, { 1, 0}, { 1, 1}
    };
    
    // Extract only boundary cells (occupied cells adjacent to free space)
    for (unsigned int y = 1; y < info.height - 1; ++y) {
        for (unsigned int x = 1; x < info.width - 1; ++x) {
            int index = y * info.width + x;
            
            if (index >= static_cast<int>(data.size())) continue;
            
            int8_t cell_value = data[index];
            
            // Only process occupied cells
            if (cell_value >= occupancy_threshold_) {
                bool is_boundary = false;
                
                // Check if any neighbor is free space
                for (const auto& [dx, dy] : neighbors) {
                    int neighbor_x = x + dx;
                    int neighbor_y = y + dy;
                    int neighbor_index = neighbor_y * info.width + neighbor_x;
                    
                    if (neighbor_index >= 0 && neighbor_index < static_cast<int>(data.size())) {
                        int8_t neighbor_value = data[neighbor_index];
                        
                        // Boundary if occupied cell is next to free space
                        if (neighbor_value >= 0 && neighbor_value < occupancy_threshold_) {
                            is_boundary = true;
                            break;
                        }
                    }
                }
                
                if (is_boundary) {
                    geometry_msgs::msg::Point world_point = gridToWorld(x, y, info);
                    boundary_points.push_back(world_point);
                }
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Extracted %zu boundary points from occupied cells", boundary_points.size());
    return boundary_points;
}


    void saveOccupiedCellsToCSV(const std::vector<geometry_msgs::msg::Point>& points)
    {
        std::ofstream csv_file(output_csv_path_);
        
        if (!csv_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", output_csv_path_.c_str());
            return;
        }

        // Write header
        csv_file << "x,y\n";
        
        // Write occupied cell coordinates with high precision
        csv_file << std::fixed << std::setprecision(6);
        for (const auto& point : points) {
            csv_file << point.x << "," << point.y << "\n";
        }
        
        csv_file.close();
        RCLCPP_INFO(this->get_logger(), "Saved %zu occupied cells to %s", points.size(), output_csv_path_.c_str());
    }

    geometry_msgs::msg::Point gridToWorld(int grid_x, int grid_y, const nav_msgs::msg::MapMetaData& info)
    {
        geometry_msgs::msg::Point world_point;
        world_point.x = info.origin.position.x + (grid_x + 0.5) * info.resolution;
        world_point.y = info.origin.position.y + (grid_y + 0.5) * info.resolution;
        world_point.z = 0.0;
        return world_point;
    }

    void publishVisualizationMarkers(const std::vector<geometry_msgs::msg::Point>& points)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Clear previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = frame_id_;
        clear_marker.header.stamp = this->now();
        clear_marker.ns = "occupied_cells";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // Create marker for occupied cells
        visualization_msgs::msg::Marker occupied_marker;
        occupied_marker.header.frame_id = frame_id_;
        occupied_marker.header.stamp = this->now();
        occupied_marker.ns = "occupied_cells";
        occupied_marker.id = 0;
        occupied_marker.type = visualization_msgs::msg::Marker::POINTS;
        occupied_marker.action = visualization_msgs::msg::Marker::ADD;
        
        occupied_marker.scale.x = 0.02;
        occupied_marker.scale.y = 0.02;
        occupied_marker.color.r = 1.0;
        occupied_marker.color.g = 0.0;
        occupied_marker.color.b = 0.0;
        occupied_marker.color.a = 0.8;
        
        occupied_marker.points = points;
        marker_array.markers.push_back(occupied_marker);
        marker_pub_->publish(marker_array);
    }

    // Members
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int occupancy_threshold_;
    std::string frame_id_;
    std::string output_csv_path_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyExtractor>();
    RCLCPP_INFO(node->get_logger(), "Starting occupancy extractor node...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
