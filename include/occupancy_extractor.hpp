#ifndef OCCUPANCY_EXTRACTOR_HPP
#define OCCUPANCY_EXTRACTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include <vector>

class OccupancyExtractor : public rclcpp::Node
{
public:
    OccupancyExtractor();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    std::vector<geometry_msgs::msg::Point> extractOccupiedCells(
        const nav_msgs::msg::OccupancyGrid::SharedPtr grid);
    geometry_msgs::msg::Point gridToWorld(int grid_x, int grid_y, 
        const nav_msgs::msg::MapMetaData& info);
    void publishVisualizationMarkers(const std::vector<geometry_msgs::msg::Point>& points);
    void publishPointCloud(const std::vector<geometry_msgs::msg::Point>& points);

    // Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    // Parameters
    int occupancy_threshold_;
    std::string frame_id_;
};

#endif // OCCUPANCY_EXTRACTOR_HPP
