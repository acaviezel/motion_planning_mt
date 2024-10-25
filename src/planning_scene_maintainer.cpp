#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PlanningSceneMaintainer : public rclcpp::Node
{
    public:
        PlanningSceneMaintainer(): Node("planning_scene_maintainer")
        {
            // Add a box to the planning scene
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

            auto const collision_object = [] {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = "panda_link0";
            collision_object.id = "bottom_plate";
            shape_msgs::msg::SolidPrimitive primitive;

            // Define the size of the box in meters
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = 1.0;
            primitive.dimensions[primitive.BOX_Y] = 1.0;
            primitive.dimensions[primitive.BOX_Z] = 0.02;

            // Populate the ground plate, so planning won't plan a path that bumps into the real ground
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = 0.0;
            box_pose.position.y = 0.0;
            box_pose.position.z = -0.05;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            return collision_object;
            }();

            
            auto const collision_object2 = [] {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = "panda_link0";
            collision_object.id = "back_wall";
            shape_msgs::msg::SolidPrimitive primitive;

            // Define the size of the box in meters
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = 0.02;
            primitive.dimensions[primitive.BOX_Y] = 1.0;
            primitive.dimensions[primitive.BOX_Z] = 1.0;

            // Define the pose of the box (relative to the frame_id)
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = -0.7;
            box_pose.position.y = 0.0;
            box_pose.position.z = 0.5;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            return collision_object;
            }();
            

            std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
            collision_objects.push_back(collision_object);
            // collision_objects.push_back(collision_object2);
            
            planning_scene_interface.applyCollisionObjects(collision_objects);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlanningSceneMaintainer>();
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
    return 0;
}