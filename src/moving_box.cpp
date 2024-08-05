#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

using namespace std::chrono_literals;

class MovingBox : public rclcpp::Node
{
public:
    MovingBox() : Node("moving_box"), updated_position(0.0), direction(1)
    {
        this->declare_parameter<std::vector<double>>("initial_position", {1.0, 0.0, 0.6});
        this->declare_parameter<double>("box_size", 0.2);
        this->declare_parameter<double>("speed", 0.5);
        this->declare_parameter<std::string>("moving_direction", "y");

        // Get parameter values
        std::vector<double> initial_position;
        this->get_parameter("initial_position", initial_position);
        if (initial_position.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Initial position must have three elements (x, y, z)");
            return;
        }
        initial_x = initial_position[0];
        initial_y = initial_position[1];
        initial_z = initial_position[2];

        this->get_parameter("box_size", box_size);
        this->get_parameter("speed", speed);
        this->get_parameter("moving_direction", moving_direction);

        // Initialize the planning scene interface
        planning_scene_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        // Set initial position
        if (moving_direction == "x")
        {
            updated_position = initial_x;
        }
        else if (moving_direction == "y")
        {
            updated_position = initial_y;
        }
        else if (moving_direction == "z")
        {
            updated_position = initial_z;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid moving direction. Use 'x', 'y', or 'z'");
            return;
        }
        

        // Add the box to the planning scene
        addBoxToPlanningScene();
        rclcpp::sleep_for(1s);

        // Create a timer to update the box position
        timer = this->create_wall_timer(40ms, std::bind(&MovingBox::updateBoxPosition, this));
    }

    void addBoxToPlanningScene()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "panda_link0";
        collision_object.id = "moving_box";

        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions.resize(3);
        box.dimensions = {box_size, box_size, box_size};

        // Define the pose of the box
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = initial_x;
        box_pose.position.y = initial_y;
        box_pose.position.z = initial_z;

        collision_object.primitives.push_back(box);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface.applyCollisionObject(collision_object);
        // Publish the planning scene message
        // moveit_msgs::msg::PlanningScene planning_scene_msg;
        // planning_scene_msg.world.collision_objects.push_back(collision_object);
        // planning_scene_msg.is_diff = true;

        // planning_scene_publisher->publish(planning_scene_msg);

        RCLCPP_INFO(this->get_logger(), "Added box to the planning scene");
    }

    void updateBoxPosition()
    {
        // Update the x position
        updated_position += direction * speed * (1 / update_rate); // (1/update_rate) seconds per update

        if (updated_position >= initial_position_by_direction() + max_range)
        {
            direction = -1;
        }
        else if (updated_position <= initial_position_by_direction() - max_range)
        {
            direction = 1;
        }

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "panda_link0";
        collision_object.id = "moving_box";

        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions.resize(3);
        box.dimensions = {box_size, box_size, box_size};

        // Define the pose of the box
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = initial_x;
        box_pose.position.y = initial_y;
        box_pose.position.z = initial_z;

        if (moving_direction == "x")
        {
            box_pose.position.x = updated_position;
        }
        else if (moving_direction == "y")
        {
            box_pose.position.y = updated_position;
        }
        else if (moving_direction == "z")
        {
            box_pose.position.z = updated_position;
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Current position: " << box_pose.position.x << " " << box_pose.position.y << " " << box_pose.position.z);

        collision_object.primitives.push_back(box);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.is_diff = true;

        planning_scene_publisher->publish(planning_scene_msg);
    }

private:
    double initial_position_by_direction() const
    {
        if (moving_direction == "x")
        {
            return initial_x;
        }
        else if (moving_direction == "y")
        {
            return initial_y;
        }
        else if (moving_direction == "z")
        {
            return initial_z;
        }
        return 0.0;
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    double box_size;
    double initial_x;
    double initial_y;
    double initial_z;
    double updated_position; // Relative position to the initial_x or initial_y or initial_z
    double speed;
    int direction; // Moving direction
    double update_rate = 25.0;
    double max_range = 0.5; // Range of movement (unit: m)
    std::string moving_direction;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovingBox>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
