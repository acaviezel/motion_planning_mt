#include <memory>
#include <string>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <atomic>
#include <thread>

using namespace std::chrono_literals;
using namespace Eigen;

// Function to get transformation matrix from DH parameters
Matrix4d get_tf_mat(int i, const std::vector<std::vector<double>>& dh) {
    double a = dh[i][0];
    double d = dh[i][1];
    double alpha = dh[i][2];
    double theta = dh[i][3];

    double q = theta;  
    Matrix4d T;
    T << std::cos(q), -std::sin(q),  0, a,
         std::sin(q) * std::cos(alpha), std::cos(q) * std::cos(alpha), -std::sin(alpha), -std::sin(alpha) * d,
         std::sin(q) * std::sin(alpha), std::cos(q) * std::sin(alpha),  std::cos(alpha), std::cos(alpha) * d,
         0, 0, 0, 1;

    return T;
}

// Function to calculate the forward kinematics solution
Matrix4d get_fk_solution(const std::vector<double>& joint_angles) {
    std::vector<std::vector<double>> dh_params = {
        {0, 0.333, 0, joint_angles[0]},
        {0, 0, -M_PI / 2, joint_angles[1]},
        {0, 0.316, M_PI / 2, joint_angles[2]},
        {0.0825, 0, M_PI / 2, joint_angles[3]},
        {-0.0825, 0.384, -M_PI / 2, joint_angles[4]},
        {0, 0, M_PI / 2, joint_angles[5]},
        {0.088, 0, M_PI / 2, joint_angles[6]},
        {0, 0.107, 0, 0},
        {0, 0, 0, -M_PI / 4},
        {0, 0.1034, 0, 0}
    };

    Matrix4d T = Matrix4d::Identity(); // Initialize as identity matrix
    for (int i = 0; i < 10; ++i) {
        T = T * get_tf_mat(i, dh_params);
    }

    return T;
}


class RRTPlanner : public rclcpp::Node
{
public:
    RRTPlanner() : Node("rrt_planner"), joint_model_group(nullptr) {}

    void initialize()
    {
        rclcpp::Node::SharedPtr node = shared_from_this();

        // Initialize MoveIt components
        move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
        visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface->getRobotModel());
        visual_tools->deleteAllMarkers();

        joint_model_group = move_group_interface->getRobotModel()->getJointModelGroup("panda_arm");
        // Ensure joint_model_group is not null
        if (!joint_model_group)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get joint model group 'panda_arm'");
            throw std::runtime_error("Failed to get joint model group 'panda_arm'");
        }

        move_group_interface->setPlanningTime(10.0);
        move_group_interface->setMaxAccelerationScalingFactor(0.3);
        move_group_interface->setMaxVelocityScalingFactor(0.3);

        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
        planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();

        this->declare_parameter<std::vector<double>>("goal_position", {0.5, 0.0, 0.6});
        this->declare_parameter<std::vector<double>>("goal_orientation", {1.0, 0.0, 0.0, 0.0}); // x, y, z, w
        this->declare_parameter<bool>("set_goal_pose",false);
        this->declare_parameter<int>("seed",1);
    }

    void executeMotionPlan()
    {
        visual_tools->deleteAllMarkers();
        geometry_msgs::msg::Pose target_pose;

        bool set_goal_pose;
        this->get_parameter("set_goal_pose", set_goal_pose);

        if (set_goal_pose)
        {
            std::vector<double> goal_position {};
            std::vector<double> goal_orientation {};
            this->get_parameter("goal_position", goal_position);
            this->get_parameter("goal_orientation", goal_orientation);

            if (goal_position.size() != 3 || goal_orientation.size() != 4)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid goal position or orientation size");
                return;
            }
            target_pose.position.x = goal_position[0];
            target_pose.position.y = goal_position[1];
            target_pose.position.z = goal_position[2];

            Eigen::Quaterniond orientation(goal_orientation[3], goal_orientation[0], goal_orientation[1], goal_orientation[2]);
            orientation.normalize();
            target_pose.orientation = tf2::toMsg(orientation);
        }
        else
        {
            // set to a position that does not collide with the obstacle
            int seed{};
            this->get_parameter("seed",seed);
            std::mt19937 gen(seed);
            srand(seed);
            random_numbers::RandomNumberGenerator rng(seed);
            bool collision_free = false;
            while(!collision_free)
            {
                
                
                moveit::core::RobotState random_state(move_group_interface->getRobotModel());
                random_state.setToRandomPositions(joint_model_group, rng);
                random_state.update();

                planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
                planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
                locked_planning_scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
                moveit::core::RobotState current_state = locked_planning_scene->getCurrentState();

                RCLCPP_INFO_STREAM(this->get_logger(), "collision detector: " << locked_planning_scene->getCollisionDetectorName());

                collision_detection::CollisionRequest collision_request;
                collision_detection::CollisionResult collision_result;
                collision_request.contacts = true;
                locked_planning_scene->getCollisionEnv()->checkRobotCollision(collision_request, collision_result, random_state, current_state);

                if (!collision_result.collision)
                {
                    // Check if the end effector is within the range before setting `collision_free` to true
                    
                    // Get the joint names
                    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

                    // Get the joint values
                    std::vector<double> joint_values;
                    random_state.copyJointGroupPositions(joint_model_group, joint_values);

                    // Print each joint's name and value
                    for (size_t i = 0; i < joint_names.size(); ++i) {
                        std::cout << "Joint '" << joint_names[i] << "': " << joint_values[i] * 57.29578 << " degrees" << std::endl;
                    }
                    Matrix4d T = get_fk_solution(joint_values);
                    Vector3d position = T.block<3,1>(0,3);
                    Quaterniond orientation(T.block<3,3>(0,0));
                    orientation.normalize();

                    if (0.0 < position.x() && 0.6 > position.x() && -0.5 < position.y() && 0.5 > position.y()
                        && 0.15 < position.z() && 0.8 > position.z())
                    {
                        collision_free = true;
                    }
                    else
                    {
                        continue;
                    }
                    // const Eigen::Isometry3d& end_effector_pose = random_state.getGlobalLinkTransform("panda_link8");
                    // target_pose.position = tf2::toMsg(Eigen::Vector3d(end_effector_pose.translation()));
                    // target_pose.orientation = tf2::toMsg(Eigen::Quaterniond(end_effector_pose.rotation()));
                    target_pose.position = tf2::toMsg(position);
                    target_pose.orientation = tf2::toMsg(orientation);
                    RCLCPP_INFO_STREAM(this->get_logger(),"from FK equation: "<< target_pose.position.x <<" "<< target_pose.position.y << " "<<target_pose.position.z);
                }
            }
        }

        visual_tools->publishAxisLabeled(target_pose, "target_pose");
        visual_tools->trigger();
        move_group_interface->setPoseTarget(target_pose,"panda_hand_tcp");

        // Plan to the target pose
        auto plan_start = std::chrono::steady_clock::now();
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools->trigger();

            RCLCPP_INFO(this->get_logger(), "Plan successful, executing the plan...");

            // Set collision counter to 0
            collision_count = 0;

            // Start collision checking in another thread
            std::thread check_collision_thread([this] {
                rclcpp::Rate rate(10); // 10 Hz collision check
                while (rclcpp::ok() && !motion_complete)  // Check until motion is done
                {
                    checkCollision();
                    rate.sleep();
                }
            });

            // Execute the motion plan
            move_group_interface->execute(my_plan);
            
            motion_complete = true;
            check_collision_thread.join();

            /*bool reached_goal = false;
            while (!reached_goal && rclcpp::ok())
            {
                checkCollision();

                // Check if the robot is close to the goal pose
                auto current_pose = move_group_interface->getCurrentPose("panda_link8");
                double position_tolerance = 0.01;
                double orientation_tolerance = 0.015;

                if (isPoseClose(current_pose.pose, target_pose, position_tolerance, orientation_tolerance))
                {
                    reached_goal = true;
                }
                rclcpp::spin_some(this->get_node_base_interface());
                rclcpp::sleep_for(100ms);
            }*/

            RCLCPP_INFO(this->get_logger(), "Motion execution completed");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan");
        }
        auto execute_end = std::chrono::steady_clock::now();

        auto plan_duration = my_plan.planning_time_;
        auto execute_duration = std::chrono::duration_cast<std::chrono::milliseconds>(execute_end - plan_start);

        std::cout << "the planner takes " << plan_duration << " s to plan" << std::endl;
        std::cout << "the whole plan takes " << execute_duration.count() / 1000.0 << " s to execute" << std::endl;
        std::cout << "number of collisions detected: " << collision_count << std::endl;

        std::ofstream outfile;
        outfile.open("/home/pinliu/ws_moveit2/data/rrt/motion_plan_durations.txt", std::ios_base::app);
        if (outfile.is_open())
        {
            outfile << plan_duration << " " << execute_duration.count() / 1000.0 << " "<< collision_count << "\n";
            outfile.close();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file for writing");
        }
    }

private:
    void checkCollision()
    {
        planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
        planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.contacts = true;
        locked_planning_scene->checkCollision(collision_request, collision_result);
        RCLCPP_INFO(this->get_logger(), "checking collision callback is called");
        if (collision_result.collision)
        {
            RCLCPP_WARN(this->get_logger(), "Robot is in collision!");
            collision_count++;
        }
    }


    const std::string PLANNING_GROUP{"panda_arm"};
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
    const moveit::core::JointModelGroup *joint_model_group;
    std::atomic<int> collision_count;
    std::atomic<bool> motion_complete {false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTPlanner>();
    node->initialize();
    node->executeMotionPlan();
    rclcpp::shutdown();
    return 0;
}
