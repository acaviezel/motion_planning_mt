#include <memory>
#include <string>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <random>
#include <random_numbers/random_numbers.h>
#include <Eigen/Dense>
#include <cmath>
#include <thread>
#include <atomic>

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

class MotionPlanner : public rclcpp::Node
{
    public:
        MotionPlanner(): Node("motion_planner"), joint_model_group(nullptr) {}

        void initialize()
        {
            rclcpp::Node::SharedPtr node = shared_from_this();

            // Initialize moveit objects
            move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
            planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
            visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node, "panda_link0", 
                                    rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface->getRobotModel());
            visual_tools->deleteAllMarkers();

            joint_model_group = move_group_interface->getRobotModel()->getJointModelGroup("panda_arm");
            move_group_interface->setPlanningTime(10.0);
            move_group_interface->setMaxAccelerationScalingFactor(0.3);
            move_group_interface->setMaxVelocityScalingFactor(0.3);

            planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
            if (!planning_scene_monitor_->getPlanningScene())
            {
                RCLCPP_ERROR(node->get_logger(), "Planning Scene Monitor did not initialize correctly");
                throw std::runtime_error("Planning Scene Monitor failed to initialize");
            }
            planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
            planning_scene_monitor_->startWorldGeometryMonitor();
            planning_scene_monitor_->startStateMonitor();

            // Set up publisher for joint trajectory and goal pose
            goal_pose_publisher = this->create_publisher<geometry_msgs::msg::Pose>(
                "goal_pose", 10);

            joint_trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "nullspace_configuration", 10);
            
            joint_trajectory_subscription = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "panda_arm_controller/joint_trajectory", 10, std::bind(&MotionPlanner::jointTrajectorySubscriptionCallback, this, std::placeholders::_1));

            this->declare_parameter<std::vector<double>>("goal_position", {0.5, 0.0, 0.6});
            this->declare_parameter<std::vector<double>>("goal_orientation", {1.0, 0.0, 0.0, 0.0});  // x, y, z, w
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

                    collision_detection::CollisionRequest collision_request;
                    collision_detection::CollisionResult collision_result;
                    collision_request.contacts = true;
                    locked_planning_scene->getCollisionEnv()->checkRobotCollision(collision_request, collision_result, random_state, current_state);

                    if (!collision_result.collision)
                    {
                        collision_free = true;
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
            move_group_interface->setPoseTarget(target_pose, "panda_hand_tcp");

            // Plan to the target pose
            auto plan_start = std::chrono::steady_clock::now();
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success)
            {
                visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
                visual_tools->trigger();

                RCLCPP_INFO(this->get_logger(), "Plan successful");
                RCLCPP_INFO_STREAM(this->get_logger(), "number of way points: " << my_plan.trajectory_.joint_trajectory.points.size());
                
                // Set collision counter to 0
                collision_count = 0;

                // Start check collision in another thread
                std::thread check_collision_thread([this] {
                    rclcpp::Rate rate(10); // 10 Hz collision check
                    while (rclcpp::ok() && !motion_complete)  
                    {
                        checkCollision();
                        rate.sleep();
                    }}
                );

                // Reference Selector logic
                auto& trajectory_points = my_plan.trajectory_.joint_trajectory.points;
                std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::iterator it = trajectory_points.begin()+1;
                std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::iterator last_it = trajectory_points.end()-1;
                
                // Initialize variables to track changes in residuals
                double previous_position_diff = std::numeric_limits<double>::max();
                double previous_angle_diff = std::numeric_limits<double>::max();
                int stable_iterations = 0;
                const int MAX_STALBE_ITERATIONS = 50;
                const double RESIDUAL_CHANGE_THRESHOLD = 1e-4;

                // Another variables to track if the maximum duration time is reached
                const auto timeout_duration = 20s;

                while (rclcpp::ok() && (std::chrono::steady_clock::now() - plan_start) < timeout_duration)
                {
                    // RCLCPP_INFO(this->get_logger(), "start do while loop");
                    std::stringstream position_str;
                    for (const auto& val: (*last_it).positions) position_str << val << " ";
                    RCLCPP_INFO_STREAM(this->get_logger(), "the last position: " << position_str.str());
                    
                    q_r.header.stamp.sec = 0;
                    q_r.header.stamp.nanosec = 0;
                    q_r.header.frame_id = "panda_link0";
                    q_r.joint_names = my_plan.trajectory_.joint_trajectory.joint_names;
                    // RCLCPP_INFO(this->get_logger(), "before using *it");
                    (*it).velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    (*it).accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    (*it).time_from_start.sec = 0;
                    (*it).time_from_start.nanosec = 0;
                    // RCLCPP_INFO(this->get_logger(), "before using q_r.points[0] = *it");
                    q_r.points.clear();
                    q_r.points.push_back(*it);
                    Matrix4d intermediate_goal_pose = get_fk_solution(it->positions);
                    Vector3d intermediate_goal_position = intermediate_goal_pose.block<3,1>(0,3);
                    Quaterniond intermediate_goal_orientation(intermediate_goal_pose.block<3,3>(0,0));
                    intermediate_goal_pose_msg.position = tf2::toMsg(intermediate_goal_position);
                    intermediate_goal_pose_msg.orientation = tf2::toMsg(intermediate_goal_orientation);
                    goal_pose_publisher->publish(intermediate_goal_pose_msg);
                    // q_r.points[0] = *it; // The fault 
                    // RCLCPP_INFO(this->get_logger(), "before publising q_r");
                    joint_trajectory_publisher->publish(q_r);
                    std::stringstream q_r_str;
                    for(const auto& val: q_r.points[0].positions) q_r_str << val << " ";
                    RCLCPP_INFO_STREAM(this->get_logger(), "q_r being published: " << q_r_str.str());
                    RCLCPP_INFO_STREAM(this->get_logger(), "number of points in q_r: " << q_r.points.size());

                    moveit::core::RobotState waypoint_state(move_group_interface->getRobotModel());
                    waypoint_state.setJointGroupPositions(joint_model_group, q_r.points[0].positions);
                    waypoint_state.update();
                    const Eigen::Isometry3d& end_effector_state = waypoint_state.getGlobalLinkTransform("panda_link8");            
                    Eigen::Vector3d end_effector_position = end_effector_state.translation();
                    visual_tools->publishSphere(end_effector_position, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
                    visual_tools->trigger();

                    RCLCPP_INFO(this->get_logger(), "right after publishing q_r");

                    // check the validity of next waypoint using continuous collision detection
                    planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
                    planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
                    locked_planning_scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());
                    moveit::core::RobotState current_state = locked_planning_scene->getCurrentState();
                    moveit::core::RobotState q_r_s1(move_group_interface->getRobotModel());
                    if (it != (trajectory_points.end()-1))
                    {
                        q_r_s1.setJointGroupPositions(joint_model_group, (*(it+1)).positions);
                        q_r_s1.update();
                    }

                    collision_detection::CollisionRequest collision_request;
                    collision_detection::CollisionResult collision_result;
                    collision_request.contacts = true;
                    locked_planning_scene->getCollisionEnv()->checkRobotCollision(collision_request, collision_result, q_r_s1, current_state);

                    if (it != (trajectory_points.end()-1))
                    {
                        if (collision_result.collision)
                        {
                            q_r.points[0] = *it;
                        }
                        else
                        {
                            q_r.points[0] = *(it+1);
                            it++;
                        }
                    }
                    
                    // Compute the residual between current pose and the goal pose
                    Eigen::Quaterniond current_rot(current_state.getGlobalLinkTransform("panda_hand_tcp").rotation());
                    Eigen::Vector3d current_pos(current_state.getGlobalLinkTransform("panda_hand_tcp").translation());
                    Eigen::Quaterniond target_rot;
                    Eigen::Vector3d target_position;
                    tf2::fromMsg(target_pose.position, target_position);
                    tf2::fromMsg(target_pose.orientation, target_rot);
                    double position_diff = (target_position - current_pos).norm();
                    Eigen::AngleAxisd aa_diff(current_rot.inverse() * target_rot);
                    double angle_diff = fabs(aa_diff.angle());                    

                    RCLCPP_INFO_STREAM(this->get_logger(), "position difference: " << position_diff);
                    RCLCPP_INFO_STREAM(this->get_logger(), "angle difference: " << angle_diff);
                    if (position_diff < 0.015 && angle_diff < 0.03)
                    {
                        break;
                    }
                    
                    // Check the change in residuals
                    double position_diff_change = fabs(position_diff - previous_position_diff);
                    double angle_diff_change = fabs(angle_diff - previous_angle_diff);

                    RCLCPP_INFO_STREAM(this->get_logger(), "position difference change: " << position_diff_change);
                    RCLCPP_INFO_STREAM(this->get_logger(), "angle difference change: " << angle_diff_change);
                    if (position_diff_change < RESIDUAL_CHANGE_THRESHOLD && angle_diff_change < RESIDUAL_CHANGE_THRESHOLD)
                    {
                        stable_iterations++;
                    }
                    else
                    {
                        stable_iterations = 0;
                    }

                    previous_position_diff = position_diff;
                    previous_angle_diff = angle_diff;

                    if (stable_iterations > MAX_STALBE_ITERATIONS)
                    {
                        RCLCPP_INFO_STREAM(this->get_logger(), "terminate due to stable residuals for " << MAX_STALBE_ITERATIONS << " iterations");
                        break;
                    }

                    rclcpp::sleep_for(40ms);

                }
                motion_complete = true;
                check_collision_thread.join();

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
            outfile.open("/home/pinliu/ws_moveit2/data/rrt-erg/motion_plan_durations.txt", std::ios_base::app);
            if (outfile.is_open())
            {
                outfile << plan_duration << " " << execute_duration.count() / 1000.0 << " " << collision_count << "\n";
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

        void jointTrajectorySubscriptionCallback(trajectory_msgs::msg::JointTrajectory msg)
        {
            q_v = msg.points[0].positions;
        }

        const std::string PLANNING_GROUP {"panda_arm"};
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
        const moveit::core::JointModelGroup* joint_model_group;
        std::atomic<int> collision_count;
        std::atomic<bool> motion_complete {false};

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pose_publisher;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_subscription;

        trajectory_msgs::msg::JointTrajectory q_r;
        geometry_msgs::msg::Pose intermediate_goal_pose_msg;
        std_msgs::msg::Float64MultiArray q_r_std_msgs;
        std::vector<double> q_v {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}; // robot initial configuration
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanner>();
    node->initialize();
    node->executeMotionPlan();
    rclcpp::shutdown();
    return 0;
}