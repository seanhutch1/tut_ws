#include "iar_dijkstra_planner/iar_dijkstra_planner.hpp"

namespace iar_dijkstra_planner
{

IarDijkstraPlanner::IarDijkstraPlanner()
// tf_(nullptr), costmap_(nullptr)
{
  tf_ = nullptr;
  costmap_ = nullptr;
}

IarDijkstraPlanner::~IarDijkstraPlanner()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type IarDijkstraPlanner",
    planner_name_.c_str()
  );
}


void 
IarDijkstraPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, //shared pointer to parent node
    std::string name, // planner name
    std::shared_ptr<tf2_ros::Buffer> tf, //tf buffer pointer
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros //shared pointer to costmap
)
{
    tf_ = tf;
    planner_name_ = name;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    parent_node_ = parent.lock();
    logger_ = parent_node_->get_logger();

    RCLCPP_INFO(
        logger_, "Configuring plugin %s of type IarDijkstraPlanner", planner_name_.c_str()
    );

}

void
IarDijkstraPlanner::activate()
{
    RCLCPP_INFO(
        logger_, "Activating plugin %s of type IarDijkstraPlanner", planner_name_.c_str()
    );
}

void
IarDijkstraPlanner::deactivate()
{
    RCLCPP_INFO(
        logger_, "Deactivating plugin %s of type IarDijkstraPlanner", planner_name_.c_str()
    );
}

void
IarDijkstraPlanner::cleanup()
{
    RCLCPP_INFO(
        logger_, "Cleaning up plugin %s of type IarDijkstraPlanner", planner_name_.c_str()
    );
}

nav_msgs::msg::Path 
IarDijkstraPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose)
{
    nav_msgs::msg::Path path;

    unsigned int mx, my;
    if(!costmap_->worldToMap(goal_pose.pose.position.x, goal_pose.pose.position.y, mx, my))
    {
        RCLCPP_WARN(logger_, "The goal pose is off the global costmap. Planning will fail.");
        return path;
    }
    else
    {   
        // Check wether the goal pose is in an lethal obstacle cell
        if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
            RCLCPP_WARN(logger_, "Failed to create a unique pose path because of obstacles");
            return path;
        }
    }
    if(!costmap_->worldToMap(start_pose.pose.position.x, start_pose.pose.position.y, mx, my))
    {
        RCLCPP_WARN(logger_, "The start pose is off the global costmap. Planning will fail.");
        return path;
    }

    path.header.stamp = parent_node_->now();
    path.header.frame_id = global_frame_;

    float interpolation_resolution = 0.1;

    int total_number_of_loop = std::hypot(goal_pose.pose.position.x - start_pose.pose.position.x, 
      goal_pose.pose.position.y - start_pose.pose.position.y)/interpolation_resolution;

    double x_increment = (goal_pose.pose.position.x - start_pose.pose.position.x)/total_number_of_loop;
    double y_increment = (goal_pose.pose.position.y - start_pose.pose.position.y)/total_number_of_loop;

    for(int i = 0; i<total_number_of_loop; ++i)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = start_pose.pose.position.x + x_increment * i;
      pose.pose.position.y = start_pose.pose.position.y + y_increment * i;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = parent_node_->now();
      pose.header.frame_id = global_frame_;
      path.poses.push_back(pose);
    }

    geometry_msgs::msg::PoseStamped goal = goal_pose;
    goal.header.stamp = parent_node_->now();
    goal.header.frame_id = global_frame_;
    path.poses.push_back(goal);

    return path;
}


}  // namespace iar_dijkstra_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(iar_dijkstra_planner::IarDijkstraPlanner, nav2_core::GlobalPlanner)