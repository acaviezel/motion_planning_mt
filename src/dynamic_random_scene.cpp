#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
#include <random>
#include <chrono>

using namespace std::chrono_literals;

class DynamicRandomScene : public rclcpp::Node
{
public:
    DynamicRandomScene() : Node("dynamic_random_scene"), planning_scene_interface()
    {

    }

    void initialize()
    {
        this->declare_parameter<int>("num_objects", 5);
        this->declare_parameter<int>("seed", 1);

        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(this->shared_from_this(), "robot_description");
        planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();

        planning_scene_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        generateAndMoveObjects();
    }

private:
    void generateAndMoveObjects()
    {
        int num_objects = this->get_parameter("num_objects").as_int();
        int seed = this->get_parameter("seed").as_int();
        
        std::mt19937 gen(seed);
        srand(seed);
        std::uniform_real_distribution<> dis_dim(0.05, 0.25); // dimension for the obstacles
        std::uniform_real_distribution<> dis_pos(-1.0, 1.0); // initial position for x or y
        std::uniform_real_distribution<> dis_pos_z(0.0, 1.0); // initial position for z
        std::uniform_real_distribution<> dis_speed(0.1, 0.3);
        std::uniform_int_distribution<> dis_dir(0, 2); // direction that obstacles move (x, y, or z direction)

        for (int i = 0; i < num_objects; ++i)
        {
            bool collision_free = false;
            while (!collision_free)
            {
                moveit_msgs::msg::CollisionObject collision_object;
                collision_object.id = "box" + std::to_string(i);
                collision_object.header.frame_id = "panda_link0";

                shape_msgs::msg::SolidPrimitive primitive;
                primitive.type = primitive.BOX;
                primitive.dimensions = {dis_dim(gen), dis_dim(gen), dis_dim(gen)};

                geometry_msgs::msg::Pose box_pose;
                box_pose.position.x = dis_pos(gen);
                box_pose.position.y = dis_pos(gen);
                box_pose.position.z = dis_pos_z(gen);
                Eigen::Quaterniond orientation = Eigen::Quaterniond::UnitRandom();
                box_pose.orientation = tf2::toMsg(orientation);

                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(box_pose);
                collision_object.operation = collision_object.ADD;

                planning_scene_interface.applyCollisionObject(collision_object);
                planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
                planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);

                collision_detection::CollisionRequest collision_request;
                collision_detection::CollisionResult collision_result;
                locked_planning_scene->checkCollision(collision_request, collision_result);
                
                if (!collision_result.collision)
                {
                    collision_free = true;
                    objects_.push_back(collision_object);
                    
                    ObjectMovement movement;
                    movement.speed = dis_speed(gen);
                    movement.direction = dis_dir(gen);
                    movement.initial_position = box_pose.position;
                    movement.current_position = box_pose.position;
                    movement.move_direction = 1;
                    movements_.push_back(movement);

                    timers_.push_back(this->create_wall_timer(40ms, [this, i]() { this->updateObjectPosition(i); }));
                }
                else
                {
                    planning_scene_interface.removeCollisionObjects(std::vector<std::string>({collision_object.id}));
                }
            }
        }
    }

    void updateObjectPosition(int object_index)
    {
        auto& obj = objects_[object_index];
        auto& movement = movements_[object_index];

        double delta = movement.speed * 0.04 * movement.move_direction; // our update rate is 25 Hz
        switch (movement.direction) {
            case 0: movement.current_position.x += delta; break;
            case 1: movement.current_position.y += delta; break;
            case 2: movement.current_position.z += delta; break;
        }

        double diff = 0; // the range that the obstacles can travel
        switch (movement.direction) {
            case 0: diff = std::abs(movement.current_position.x - movement.initial_position.x); break;
            case 1: diff = std::abs(movement.current_position.y - movement.initial_position.y); break;
            case 2: diff = std::abs(movement.current_position.z - movement.initial_position.z); break;
        }
        if (diff > 0.4) {
            movement.move_direction *= -1;
        }

        obj.primitive_poses[0].position = movement.current_position;

        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.world.collision_objects.push_back(obj);
        planning_scene_msg.is_diff = true;
        planning_scene_publisher->publish(planning_scene_msg);
    }

    struct ObjectMovement {
        double speed;
        int direction; // direction of x, y, or z (0, 1, or 2)
        geometry_msgs::msg::Point initial_position;
        geometry_msgs::msg::Point current_position;
        int move_direction; // either 1 or -1, indicates moving forward and backward
    };

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    std::vector<moveit_msgs::msg::CollisionObject> objects_;
    std::vector<ObjectMovement> movements_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicRandomScene>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}