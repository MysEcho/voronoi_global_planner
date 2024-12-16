#include "voronoi_global_planner/voronoi_global_planner.hpp"
#include <nav2_util/node_utils.hpp>

namespace voronoi_global_planner {

void VoronoiGlobalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent;
    name_ = name;
    tf_buffer_ = tf_buffer;
    costmap_ros_ = costmap_ros;

    // Extract obstacles from costmap
    std::vector<Point> obstacles = extractObstaclesFromCostmap();
    
    // Generate Voronoi diagram
    voronoi_diagram_.generateVoronoiDiagram(obstacles);

    RCLCPP_INFO(
        node_->get_logger(), 
        "Configured Voronoi Global Planner: %s", 
        name.c_str()
    );
}

void VoronoiGlobalPlanner::cleanup() {
    RCLCPP_INFO(
        node_->get_logger(), 
        "Cleaning up Voronoi Global Planner: %s", 
        name_.c_str()
    );
}

void VoronoiGlobalPlanner::activate() {
    RCLCPP_INFO(
        node_->get_logger(), 
        "Activating Voronoi Global Planner: %s", 
        name_.c_str()
    );
}

void VoronoiGlobalPlanner::deactivate() {
    RCLCPP_INFO(
        node_->get_logger(), 
        "Deactivating Voronoi Global Planner: %s", 
        name_.c_str()
    );
}

nav_msgs::msg::Path VoronoiGlobalPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = costmap_ros_->getGlobalFrameID();

    // Convert start and goal to Points
    Point start_point = poseToPoint(start);
    Point goal_point = poseToPoint(goal);

    // Find Voronoi vertices near start and goal
    Point start_voronoi = voronoi_diagram_.findNearestVoronoiVertex(start_point);
    Point goal_voronoi = voronoi_diagram_.findNearestVoronoiVertex(goal_point);

// Create a path through Voronoi vertices
    std::vector<Point> voronoi_path = {start_point, start_voronoi};
    
    // Simple path interpolation through Voronoi vertices
    voronoi_path.push_back(goal_voronoi);
    voronoi_path.push_back(goal_point);

    // Check path validity
    if (!voronoi_diagram_.isValidPath(voronoi_path)) {
        RCLCPP_WARN(
            node_->get_logger(), 
            "Unable to find a valid Voronoi path between start and goal"
        );
        return path;
    }

    // Convert Voronoi path to ROS path
    for (const auto& point : voronoi_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        
        // Set orientation (simplified)
        pose.pose.orientation.w = 1.0;
        
        path.poses.push_back(pose);
    }

    return path;
}

std::vector<Point> VoronoiGlobalPlanner::extractObstaclesFromCostmap() {
    std::vector<Point> obstacles;
    
    // Get costmap information
    auto* costmap = costmap_ros_->getCostmap();
    unsigned int mx, my;
    
    // Threshold for considering a cell as an obstacle
    unsigned char obstacle_threshold = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    
    for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
        for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
            // Check if cell is an obstacle
            if (costmap->getCost(i, j) >= obstacle_threshold) {
                // Convert map coordinates to world coordinates
                double wx, wy;
                costmap->mapToWorld(i, j, wx, wy);
                
                obstacles.emplace_back(wx, wy);
            }
        }
    }
    
    RCLCPP_INFO(
        node_->get_logger(), 
        "Extracted %zu obstacles from costmap", 
        obstacles.size()
    );
    
    return obstacles;
}

Point VoronoiGlobalPlanner::poseToPoint(const geometry_msgs::msg::PoseStamped& pose) {
    return Point(pose.pose.position.x, pose.pose.position.y);
}

} // namespace voronoi_global_planner