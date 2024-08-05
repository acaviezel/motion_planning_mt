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

class RandomSceneGenerator : public rclcpp::Node
{
    public:
        RandomSceneGenerator() : Node("random_scene_generator"), planning_scene_interface() { }

        void initialize()
        {
            rclcpp::Node::SharedPtr node = shared_from_this();
            planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
            planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
            planning_scene_monitor_->startStateMonitor();
            planning_scene_monitor_->startWorldGeometryMonitor();
            this->declare_parameter<int>("num_objects", 5); // Default number is 5
            this->declare_parameter<int>("seed", 1); // Default number is 1
        }

        void generateRandomCollisionObjects()
        {
            int num_objects = this->get_parameter("num_objects").as_int();
            int seed = this->get_parameter("seed").as_int();
            std::mt19937 gen(seed);
            srand(seed);
            std::uniform_real_distribution<> dis_dim(0.0, 0.5);
            std::uniform_real_distribution<> dis_pos(-1.0, 1.0);
            std::uniform_real_distribution<> dis_pos_z(0.0, 1.0);

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
                    }
                    else
                    {
                        // Remove the object because it collides with the robot, and try the next random object.
                        planning_scene_interface.removeCollisionObjects(std::vector<std::string>({collision_object.id}));
                    }
                }
            }
        }
    
    private:
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomSceneGenerator>();
    node->initialize();
    node->generateRandomCollisionObjects();
    rclcpp::sleep_for(std::chrono::seconds(5));
    rclcpp::shutdown();
    return 0;
}