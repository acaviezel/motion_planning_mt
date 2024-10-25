#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class GuidedMovingBox : public rclcpp::Node
{
public:
    GuidedMovingBox() : Node("guided_moving_box"), planning_scene_interface()
    {
        this->declare_parameter<double>("speed", 0.2);
    }

    void initialize()
    {
        rclcpp::Node::SharedPtr node = shared_from_this();
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
        planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();

        planning_scene_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        createObstacle();
        initializeMovement();
    }

private:
    void createObstacle()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "moving_cube";
        collision_object.header.frame_id = "panda_link0";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.1, 0.1, 0.1};  // Cube of length 0.1

        geometry_msgs::msg::Pose cube_pose;
        cube_pose.position.x = 0.0;
        cube_pose.position.y = 0.5;
        cube_pose.position.z = 1.0;
        cube_pose.orientation.w = 1.0;  // No rotation

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(cube_pose);
        collision_object.operation = collision_object.ADD;

        obstacle_ = collision_object;
        initial_position_ = cube_pose.position;
        current_position_ = initial_position_;

        planning_scene_interface.applyCollisionObject(collision_object);
    }

    void initializeMovement()
    {
        planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
        planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(planning_scene_monitor_);

        if (!locked_planning_scene)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get the planning scene");
            return;
        }

        moveit::core::RobotState current_state = locked_planning_scene->getCurrentState();
        Eigen::Vector3d link5_position = current_state.getGlobalLinkTransform("panda_link5").translation();

        target_position_.x = link5_position.x();
        target_position_.y = link5_position.y();
        target_position_.z = link5_position.z();

        // Calculate direction vector
        direction_vector.x = target_position_.x - initial_position_.x;
        direction_vector.y = target_position_.y - initial_position_.y;
        direction_vector.z = target_position_.z - initial_position_.z;
        
        // Normalize direction vector
        double length = std::sqrt(direction_vector.x * direction_vector.x + 
                                  direction_vector.y * direction_vector.y + 
                                  direction_vector.z * direction_vector.z);
        direction_vector.x /= length;
        direction_vector.y /= length;
        direction_vector.z /= length;

        forward_or_not = 1;  // Start by moving towards the target
        this->get_parameter("speed", speed_);

        timer_ = this->create_wall_timer(50ms, std::bind(&GuidedMovingBox::updateObstaclePosition, this));
    }

    void updateObstaclePosition()
    {
        double step = speed_ * 0.05;  // 50ms timer

        // Update position
        current_position_.x += direction_vector.x * step * forward_or_not;
        current_position_.y += direction_vector.y * step * forward_or_not;
        current_position_.z += direction_vector.z * step * forward_or_not;

        // Check if we've reached or passed the target or initial position
        double distance_to_target = calculateDistance(current_position_, target_position_);
        double distance_to_initial = calculateDistance(current_position_, initial_position_);
        double total_distance = calculateDistance(initial_position_, target_position_);

        if ((forward_or_not == 1 && distance_to_target <= step) ||
            (forward_or_not == -1 && distance_to_initial <= step))
        {
            forward_or_not *= -1;  // Change direction
        }

        obstacle_.primitive_poses[0].position = current_position_;

        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.world.collision_objects.push_back(obstacle_);
        planning_scene_msg.is_diff = true;
        planning_scene_publisher->publish(planning_scene_msg);
    }

    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    moveit_msgs::msg::CollisionObject obstacle_;
    geometry_msgs::msg::Point current_position_;
    geometry_msgs::msg::Point target_position_;
    geometry_msgs::msg::Point initial_position_;
    geometry_msgs::msg::Vector3 direction_vector;
    int forward_or_not;
    double speed_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GuidedMovingBox>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}