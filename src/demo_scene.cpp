#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

class MovingBox
{
public:
    MovingBox(
        rclcpp::Node::SharedPtr node,
        const std::string& box_name,
        const std::array<double, 3>& initial_position,
        double box_size,
        double speed,
        const std::string& moving_direction,
        double update_rate = 50.0,
        double max_range = 0.5
    ) : node_(node),
        box_name_(box_name),
        initial_position_(initial_position),
        box_size_(box_size),
        speed_(speed),
        moving_direction_(moving_direction),
        update_rate_(update_rate),
        max_range_(max_range),
        updated_position_(0.0),
        direction_(1)
    {
        planning_scene_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        if (moving_direction_ == "x")
            updated_position_ = initial_position_[0];
        else if (moving_direction_ == "y")
            updated_position_ = initial_position_[1];
        else if (moving_direction_ == "z")
            updated_position_ = initial_position_[2];
        else
            throw std::invalid_argument("Invalid moving direction. Use 'x', 'y', or 'z'");

        addBoxToPlanningScene();
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / update_rate_)),
            std::bind(&MovingBox::updateBoxPosition, this)
        );
    }

private:
    void addBoxToPlanningScene()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "fr3_link0";
        collision_object.id = box_name_;

        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = {box_size_, box_size_, box_size_};

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = initial_position_[0];
        box_pose.position.y = initial_position_[1];
        box_pose.position.z = initial_position_[2];

        collision_object.primitives.push_back(box);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_.applyCollisionObject(collision_object);

        RCLCPP_INFO(node_->get_logger(), "Added box %s to the planning scene", box_name_.c_str());
    }

    void updateBoxPosition()
    {
        updated_position_ += direction_ * speed_ * (1 / update_rate_);

        if (updated_position_ >= initial_position_by_direction() + max_range_)
            direction_ = -1;
        else if (updated_position_ <= initial_position_by_direction() - max_range_)
            direction_ = 1;

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "panda_link0";
        collision_object.id = box_name_;

        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = {box_size_, box_size_, box_size_};

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = initial_position_[0];
        box_pose.position.y = initial_position_[1];
        box_pose.position.z = initial_position_[2];

        if (moving_direction_ == "x")
            box_pose.position.x = updated_position_;
        else if (moving_direction_ == "y")
            box_pose.position.y = updated_position_;
        else if (moving_direction_ == "z")
            box_pose.position.z = updated_position_;

        collision_object.primitives.push_back(box);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.world.collision_objects.push_back(collision_object);
        planning_scene_msg.is_diff = true;

        planning_scene_publisher_->publish(planning_scene_msg);
    }

    double initial_position_by_direction() const
    {
        if (moving_direction_ == "x")
            return initial_position_[0];
        else if (moving_direction_ == "y")
            return initial_position_[1];
        else if (moving_direction_ == "z")
            return initial_position_[2];
        return 0.0;
    }

    rclcpp::Node::SharedPtr node_;
    std::string box_name_;
    std::array<double, 3> initial_position_;
    double box_size_;
    double speed_;
    std::string moving_direction_;
    double update_rate_;
    double max_range_;
    double updated_position_;
    int direction_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("demo_scene");

    // Vector to store MovingBox instances
    std::vector<std::unique_ptr<MovingBox>> boxes;

    // Final Demo scenario
    boxes.push_back(std::make_unique<MovingBox>(
        node, "box1",
        std::array<double, 3>{0.3, -0.7, 0.5}, 0.1, 0.4, "y", 50.0, 0.4
    ));

    boxes.push_back(std::make_unique<MovingBox>(
        node, "box2",
        std::array<double, 3>{-0.5, 0.3, 0.8}, 0.1, 0.4, "x"
    ));

    boxes.push_back(std::make_unique<MovingBox>(
        node, "box3",
        std::array<double, 3>{0.4, 0.6, 0.95}, 0.1, 0.4, "z"
    ));

    // Oscillation scenario
    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    // auto box1 = std::make_unique<MovingBox>(
    //     node, "box1",
    //     std::array<double, 3>{-0.2, -0.4, 0.6}, 0.1, 0.4, "y", 50.0, 0.3
    // );

    // // Spin the executor for 3 seconds
    // auto start_time = node->now();
    // while (rclcpp::ok() && (node->now() - start_time).seconds() < 0.33) {
    //     executor.spin_some();
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }    

    // auto box2 = std::make_unique<MovingBox>(
    //     node, "box2",
    //     std::array<double, 3>{0.3,-0.4, 0.6}, 0.1, 0.4, "y", 50.0, 0.3
    // );


    RCLCPP_INFO(node->get_logger(), "Created moving boxes successfully.");

    // executor.spin();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}