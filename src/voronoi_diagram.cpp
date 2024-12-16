#ifndef VORONOI_GLOBAL_PLANNER__VORONOI_GLOBAL_PLANNER_HPP_
#define VORONOI_GLOBAL_PLANNER__VORONOI_GLOBAL_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_loader.hpp>

#include "/voronoi_global_planner/voronoi_diagram.hpp"

namespace voronoi_global_planner {

class VoronoiGlobalPlanner : public nav2_core::GlobalPlanner {
public:
    VoronoiGlobalPlanner() = default;
    ~VoronoiGlobalPlanner() = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    
    VoronoiDiagram voronoi_diagram_;
    
    // Convert map to obstacle points
    std::vector<Point> extractObstaclesFromCostmap();
    
    // Helper method to convert between ROS poses and Points
    Point poseToPoint(const geometry_msgs::msg::PoseStamped& pose);
};

} // namespace voronoi_global_planner

#endif // VORONOI_GLOBAL_PLANNER__VORONOI_GLOBAL_PLANNER_HPP_