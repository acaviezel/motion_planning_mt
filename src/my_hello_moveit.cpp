#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Every node need a logger
static const rclcpp::Logger LOGGER = rclcpp::get_logger("my_hello_moveit_logger");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("my_hello_moveit", node_options);

    // spin a singleThreadedExecutor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor](){executor.spin(); }).detach();

    // START OUR ACTUAL PROGRAM.

    static const std::string PLANNING_GROUP = "panda_arm";
    
    // create move group interface
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);
    
    // create planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    // create raw pointer to joint model group
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // set up visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", rvt::RVIZ_MARKER_TOPIC,
                                                        move_group_interface.getRobotModel());
    
    visual_tools.deleteAllMarkers();

    // remote control is useful that allows us to use "next" button in RViz GUI
    visual_tools.loadRemoteControl();
    
    // populate the text in RViz
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, " Hello_world_starts_now", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plot the goal pose");
    
    // Set a goal pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 0.0;
    target_pose.orientation.y = 1.0;
    target_pose.position.x = 0.28;
    target_pose.position.y = 0.5;
    target_pose.position.z = 0.5;
    move_group_interface.setPoseTarget(target_pose);
    visual_tools.publishAxisLabeled(target_pose, "goal_pose");
    visual_tools.trigger();
    
    visual_tools.prompt("Press 'next' in the RvizVusalToolsGui window to plan");
    
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);


    // create a plan object and then call the planner to plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "plan 1 %s", success ? "SUCCESS" : "FAILED");

    // visualize the plan as a line
    RCLCPP_INFO(LOGGER, "visualizing plan 1 as a line");
    visual_tools.publishText(text_pose, "demo_1", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // execute the planned trajectory
    visual_tools.prompt("Press 'next' to execute the plan.");
    move_group_interface.execute(my_plan);

    visual_tools.prompt("Press 'next' to exit the program.");
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}

