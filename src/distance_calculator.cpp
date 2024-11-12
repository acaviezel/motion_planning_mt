#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_common.h>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

#include <franka/robot_state.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/exception.h>

/*
class DistanceCalculator : public rclcpp::Node
{
public:
    DistanceCalculator() : Node("distance_calculator_node")
    {
        // Initialize the Planning Scene Monitor
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

        if (!planning_scene_monitor_->getPlanningScene())
        {
            RCLCPP_ERROR(this->get_logger(), "Planning Scene Monitor did not initialize correctly");
            throw std::runtime_error("Planning Scene Monitor failed to initialize");
        }

        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();

        // Set up a timer to compute the distance periodically
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DistanceCalculator::computeDistance, this));
    }

private:
    void computeDistance()
    {
        // Ensure the planning scene is up-to-date
        planning_scene_monitor_->requestPlanningSceneState();

        // Use LockedPlanningSceneRO for safe access
        planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

        if (!planning_scene)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get the planning scene");
            return;
        }

        // Get the current state of the robot
        moveit::core::RobotState current_state = planning_scene->getCurrentState();
        const moveit::core::LinkModel* link_model = current_state.getLinkModel("panda_hand"); // Adjust this to your end-effector link

        if (!link_model)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get the end-effector link model");
            return;
        }

        // Compute distance to all collision objects
        collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
        double min_distance = std::numeric_limits<double>::max();

        for (const auto& object_id : planning_scene->getWorld()->getObjectIds())
        {
            collision_detection::CollisionRequest collision_request;
            collision_request.contacts = true;
            collision_request.max_contacts = 1;
            collision_request.distance = true;

            collision_detection::CollisionResult collision_result;
            planning_scene->checkCollision(collision_request, collision_result, current_state, acm);

            if (collision_result.distance < min_distance)
            {
                min_distance = collision_result.distance;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Minimum distance between end-effector and obstacles: %f", min_distance);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};
*/
using namespace std::chrono_literals;

inline void dampedPseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);   
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
     S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

std::vector<double> vectorXdToStdVector(const Eigen::VectorXd& eigen_vec) {
  std::vector<double> std_vec(7);
  for (int i = 0; i < 7; ++i) {
    std_vec[i] = eigen_vec(i);
  }
  return std_vec;
}

// This functions is (for now) only experimental. This function may not be used.
double calculateRepulsivePotential(double distance, double eta = 2.0, double rho0 = 0.5) {
    // Check if the robot is within the influence range of the obstacle
    if (distance <= rho0 && distance >= 0.0) {
        // Calculate the repulsive potential
        return eta * std::pow((1.0 / distance - 1.0 / rho0), 0.3);
    } else {
        // Outside the influence range, potential is zero
        return 0.0;
    }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("distance_calculator", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    
    
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ = 
                            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    
    if (!planning_scene_monitor_->getPlanningScene())
    {
        RCLCPP_ERROR(node->get_logger(), "Planning Scene Monitor did not initialize correctly");
        throw std::runtime_error("Planning Scene Monitor failed to initialize");
    }

    // Start listening to ROS topic
    planning_scene_monitor_->startSceneMonitor("/monitored_planning_scene");
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();

    auto rviz_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>("minimum_distance_visualization",10);
    auto repulsion_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("primitive_shape_repulsion_field",10);
    rclcpp::Rate loop_rate(25);

    while (rclcpp::ok())
    {
        // Just want to check how long it takes to run the whole while loop, I can delete this afterwards
        auto start_time = std::chrono::steady_clock::now();

        // Ensure the planning scene is up-to-date
        planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
        
        // Use LockedPlanningSceneRO for safe access
        planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);

        if (!locked_planning_scene)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get the planning scene");
            return 1;
        }
        
        // Use Bullet Collision Detector so that we can calculate distance
        locked_planning_scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create());

        // Get the current state of the robot
        moveit::core::RobotState current_state = locked_planning_scene->getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");

        if (!joint_model_group)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get the joint model group");
            return 1;
        }

        // Compute distance to all collision objects
        collision_detection::AllowedCollisionMatrix acm = locked_planning_scene->getAllowedCollisionMatrix();
        
        //acm.setEntry("panda_link0",true);
        //acm.setEntry("panda_link1",true);

        // Print out the ACM matrix
        //acm.print(std::cout);

        collision_detection::CollisionRequest collision_request;
        collision_request.contacts = true;
        collision_request.max_contacts_per_pair = 1;
        collision_request.distance = true;

        collision_detection::CollisionResult collision_result;
        locked_planning_scene->checkCollision(collision_request, collision_result, current_state, acm);

        // sanity check: print know objects, print collision detector, print collision state
        locked_planning_scene->printKnownObjects();
        RCLCPP_INFO_STREAM(node->get_logger(), "collision detector: " << locked_planning_scene->getCollisionDetectorName());
        RCLCPP_INFO_STREAM(node->get_logger(), "Current state is " << (collision_result.collision ? "in" : "not in" ) << " collision.");

        
        // Calculate distance between robot and obstacles
        collision_detection::CollisionResult::ContactMap::const_iterator it;
        visualization_msgs::msg::MarkerArray mkarray;  // For visualization in RViz
        
        double min_distance = std::numeric_limits<double>::max();
        std::string min_distance_pair_first, min_distance_pair_second;
        Eigen::Vector3d min_distance_pair_first_pos, min_distance_pair_second_pos;
        Eigen::Vector3d repulsion_direction;

        for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
        {
            // std::cout << "debugging!! " <<it->second[0].body_name_1 << " " << it->second[0].body_name_2 <<"\n";
            // We only consider the distance between obstacles and links
            if((it->second[0].body_type_1 == collision_detection::BodyType::ROBOT_LINK)
                                &&(it->second[0].body_type_2 == collision_detection::BodyType::ROBOT_LINK))
            {
                continue;
            }
            // We exclude the distance from panda_link0 and panda_link1
            if((it->second[0].body_name_1 == "panda_link0") || it->second[0].body_name_1 == "panda_link1")
            {
                continue;
            }

            double distance = it->second[0].depth;
            if (distance < min_distance)
            {
                min_distance = distance;
                min_distance_pair_first = it->second[0].body_name_1;
                min_distance_pair_second = it->second[0].body_name_2;
                min_distance_pair_first_pos = it->second[0].nearest_points[0];
                min_distance_pair_second_pos = it->second[0].nearest_points[1];
                repulsion_direction = it->second[0].nearest_points[0] - it->second[0].nearest_points[1];
            }

        }

        visualization_msgs::msg::Marker spheres;
        spheres.header.frame_id = "panda_link0";
        spheres.header.stamp = node->now();
        spheres.id = 1;
        spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        spheres.action = visualization_msgs::msg::Marker::ADD;
        spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.02;
        spheres.color.r = 0.0;
        spheres.color.g = 1.0;
        spheres.color.b = 0.0;
        spheres.color.a = 0.7;
        spheres.lifetime = rclcpp::Duration(40ms);

        geometry_msgs::msg::Point p1 = tf2::toMsg(min_distance_pair_first_pos);
        geometry_msgs::msg::Point p2 = tf2::toMsg(min_distance_pair_second_pos);
        spheres.points.push_back(p1);
        spheres.points.push_back(p2);

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "panda_link0";
        arrow.header.stamp = node->now();
        arrow.id = 2;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.scale.x = 0.01; // shaft diameter
        arrow.scale.y = 0.015; // header diameter
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        arrow.lifetime = rclcpp::Duration(40ms);
        arrow.points.push_back(p1);
        arrow.points.push_back(p2);
        
        mkarray.markers.push_back(spheres);
        mkarray.markers.push_back(arrow);
        rviz_publisher->publish(mkarray);

        Eigen::Vector3d joint_i1, joint_i;  // joint position in 3D space
        double lambda; // value between [0, 1], represent the distance proportion from joint i to joint i+1
        Eigen::Matrix<double, 6, 7> J_i, J_i1; // Full Jacobian
        Eigen::MatrixXd J_i_dynamic_size;
        Eigen::Matrix<double, 3, 7> J_i_trans, J_i1_trans; // Translation part of the Jacobian
        Eigen::Matrix<double, 7, 3> J_i_trans_pinv, J_i1_trans_pinv; // Pseudo-inverse of the translation Jacobian
        Eigen::Vector3d reference_point {0.0, 0.0, 0.0};

        // Define a map for calculating jacobian given the closest link to the obstacle
        /*std::map<std::string, std::pair<std::string, std::string>> link_map = {
            {"panda_link2", {"panda_link2", "panda_link3"}},
            {"panda_link3", {"panda_link3", "panda_link4"}},
            {"panda_link4", {"panda_link4", "panda_link5"}},
            {"panda_link5", {"panda_link4", "panda_link5"}},
            {"panda_link6", {"panda_link6", "panda_link7"}},
            {"panda_link7", {"panda_link7", "panda_link8"}},
            {"panda_hand",  {"panda_link7", "panda_link8"}},
            {"panda_leftfinger", {"panda_link7", "panda_link8"}},
            {"panda_rightfinger", {"panda_link7", "panda_link8"}}
        };*/

        std::map<std::string, std::string> link_map = {
            {"panda_link2", "panda_link2"},
            {"panda_link3", "panda_link3"},
            {"panda_link4", "panda_link3"},
            {"panda_link5", "panda_link5"},
            {"panda_link6", "panda_link6"},
            {"panda_link7", "panda_link7"},
            {"panda_link8", "panda_link8"},
            {"panda_hand",  "panda_link8"},
            {"panda_leftfinger", "panda_link8"},
            {"panda_rightfinger", "panda_link8"}
        };

        auto link_it = link_map.find(min_distance_pair_first);
        if (link_it != link_map.end())
        {
            const auto& closest_link_name = link_it->second;
            joint_i = current_state.getGlobalLinkTransform(closest_link_name).translation();
            current_state.getJacobian(joint_model_group, current_state.getLinkModel(closest_link_name), reference_point, J_i_dynamic_size);
            if (J_i_dynamic_size.rows() != 6 || J_i_dynamic_size.cols() != 7)
            {
                throw std::runtime_error("Matrix size is incorrect. Expected 6x7.");
            }
            J_i = J_i_dynamic_size;
            // Extract the translation part of the Jacobian
            J_i_trans = J_i.block<3, 7>(0, 0, 3, J_i.cols());

            // Compute the pseudo inverse (damped or not damped)
            // J_i_trans_pinv = J_i_trans.completeOrthogonalDecomposition().pseudoInverse();
            double rho {0.2}; // damping term
            J_i_trans_pinv = J_i_trans.transpose()*(J_i_trans * J_i_trans.transpose() + rho*rho*Eigen::Matrix3d::Identity()).completeOrthogonalDecomposition().pseudoInverse();
        }

        // RCLCPP_INFO(node->get_logger(), "closest link: %s", min_distance_pair_first.c_str());
        // RCLCPP_INFO(node->get_logger(), "Pseudo inverse of jacobian i: ");
        // RCLCPP_INFO_STREAM(node->get_logger(), J_i_trans_pinv);
        // RCLCPP_INFO(node->get_logger(), "Pseudo inverse of jacobian i+1: ");
        // RCLCPP_INFO_STREAM(node->get_logger(), J_i1_trans_pinv);

        Eigen::VectorXd numerator;
        numerator = J_i_trans_pinv * repulsion_direction;
        double denominator;
        double eta {0.001}; // smoothing parameter
        denominator = std::max(numerator.norm(), eta);
        Eigen::VectorXd repulsion_direction_joint_space = numerator / denominator;

        // Finally combine all above to get the repulsion field in joint space
        double zeta {0.5}; // the influence margin
        double delta {0.25}; // the static safety margin
        // Eigen::VectorXd repulsion_field = calculateRepulsivePotential(min_distance) * repulsion_direction_joint_space;
        Eigen::VectorXd repulsion_field = std::max((zeta - min_distance)/(zeta - delta), 0.0) * repulsion_direction_joint_space;
        std_msgs::msg::Float64MultiArray repulsion_msg;
        repulsion_msg.data = vectorXdToStdVector(repulsion_field);
        repulsion_publisher->publish(repulsion_msg);

        RCLCPP_INFO(node->get_logger(), "number of element in contact map: %i" ,(int)collision_result.contacts.size());
        RCLCPP_INFO(node->get_logger(), "repulsion direction: (%.4f, %.4f, %.4f)" , repulsion_direction.x(), repulsion_direction.y(), repulsion_direction.z());
        RCLCPP_INFO_STREAM(node->get_logger(), "the closest link is: " << min_distance_pair_first);
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO_STREAM(node->get_logger(), "time taken to run the loop: " << elapsed_time.count() / 1000.0 << " seconds");

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
