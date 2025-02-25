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
#include <messages_fr3/msg/closest_point.hpp>
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

    //create publisher for closest distance
    auto closest_distance_publisher = node->create_publisher<messages_fr3::msg::ClosestPoint>("closest_point", 10);

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
    Eigen::Vector3d reference_point{0.0, 0.0, 0.0}; // Reference point in the link frame


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
        const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("fr3_arm");
        const moveit::core::JointModelGroup* joint_model_group_manipulator = current_state.getJointModelGroup("fr3_manipulator");
        
        

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
        /*locked_planning_scene->printKnownObjects();
        RCLCPP_INFO_STREAM(node->get_logger(), "collision detector: " << locked_planning_scene->getCollisionDetectorName());
        RCLCPP_INFO_STREAM(node->get_logger(), "Current state is " << (collision_result.collision ? "in" : "not in" ) << " collision.");*/

        
        // Calculate distance between robot and obstacles
        collision_detection::CollisionResult::ContactMap::const_iterator it;
        visualization_msgs::msg::MarkerArray mkarray;  // For visualization in RViz
        
        std::vector<std::string> links_to_check = {"fr3_link2", "fr3_link3", "fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7", "fr3_hand", "fr3_leftfinger", "fr3_rightfinger"};
        std::map<std::string, double> min_distances_per_link;

        double min_distance = std::numeric_limits<double>::max();
        std::string min_distance_pair_first, min_distance_pair_second;
        Eigen::Vector3d min_distance_pair_first_pos, min_distance_pair_second_pos;
        Eigen::Vector3d repulsion_direction;
        std::map<std::string, Eigen::Vector3d> repulsion_directions_per_link;

        // Initialize link map for all relevant links
        std::map<std::string, std::string> link_map = {
            {"fr3_link2", "fr3_link2"},
            {"fr3_link3", "fr3_link3"},
            {"fr3_link4", "fr3_link4"},
            {"fr3_link5", "fr3_link5"},
            {"fr3_link6", "fr3_link6"},
            {"fr3_link7", "fr3_link7"},
            {"fr3_link8", "fr3_link8"},
            {"fr3_hand", "fr3_link8"},
            {"fr3_leftfinger", "fr3_link8"},
            {"fr3_rightfinger", "fr3_link8"},
        };

        // Initialize data structures for storing results
        
        std::map<std::string, Eigen::VectorXd> repulsion_fields_per_link;
        std::map<std::string, std::tuple<double, double, double>> safe_positions_per_link; // To store safe x, y, z for each link

        // Variables for minimum distance per link
        // Declare dynamic vectors for each coordinate of each link/hand.
        std::vector<double> link2x;
        std::vector<double> link2y;
        std::vector<double> link2z;

        std::vector<double> link3x;
        std::vector<double> link3y;
        std::vector<double> link3z;

        std::vector<double> link4x;
        std::vector<double> link4y;
        std::vector<double> link4z;

        std::vector<double> link5x;
        std::vector<double> link5y;
        std::vector<double> link5z;

        std::vector<double> link6x;
        std::vector<double> link6y;
        std::vector<double> link6z;

        std::vector<double> link7x;
        std::vector<double> link7y;
        std::vector<double> link7z;

        std::vector<double> handx;
        std::vector<double> handy;
        std::vector<double> handz;

        std::vector<double> hand_tcp_x;
        std::vector<double> hand_tcp_y;
        std::vector<double> hand_tcp_z;

        //variables for jacobian of the closest point on each link with variable sized vectors
        std::vector<double> jacobian_array2;
        std::vector<double> jacobian_array3;
        std::vector<double> jacobian_array4;
        std::vector<double> jacobian_array5;
        std::vector<double> jacobian_array6;
        std::vector<double> jacobian_array7;
        std::vector<double> jacobian_array_hand;
        std::vector<double> jacobian_array_hand_tcp;

        // Retrieve and count the collision objects in the scene.
        const auto& collision_objects = locked_planning_scene->getWorld()->getObjectIds();
        //print collision objects id
        size_t num_objects = collision_objects.size();
        
        
        for (const auto& object : collision_objects){
            for (const auto &[link_name, closest_link] : link_map)
            {
                //RCLCPP_INFO(node->get_logger(), "Processing link: %s", link_name.c_str());

                double min_distance = std::numeric_limits<double>::max();
                Eigen::Vector3d min_distance_pair_first_pos, min_distance_pair_second_pos, repulsion_direction;

                // Loop over contacts in the collision result
                for (auto it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
                {
                    // Skip if:
                    // - contact is between two robot links,
                    // - the contact does not involve the current link, or
                    // - the contact does not involve the current obstacle.
                    if ((it->second[0].body_type_1 == collision_detection::BodyType::ROBOT_LINK &&
                        it->second[0].body_type_2 == collision_detection::BodyType::ROBOT_LINK) ||
                        it->second[0].body_name_1 != link_name ||
                        it->second[0].body_name_2 != object||
                        (object == "cylinder0" && link_name != "fr3_leftfinger"))
                    {
                        continue;
                    }

                    double distance = it->second[0].depth;
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                        min_distance_pair_first_pos = it->second[0].nearest_points[0];
                        min_distance_pair_second_pos = it->second[0].nearest_points[1];
                        repulsion_direction = min_distance_pair_first_pos - min_distance_pair_second_pos;
                    }
                }

                /*if (min_distance == std::numeric_limits<double>::max())
                {
                    // No contact for this link
                    RCLCPP_INFO(node->get_logger(), "No contact for link: %s", link_name.c_str());
                    continue;
                }*/

                // Compute the Jacobian and pseudo-inverse for the closest link
                Eigen::Matrix<double, 6, 7> J_i;
                Eigen::Matrix<double, 3, 7> J_i_trans;
                Eigen::Matrix<double, 7, 3> J_i_trans_pinv;
                Eigen::MatrixXd J_i_dynamic_size;

                Eigen::Vector3d joint_i = current_state.getGlobalLinkTransform(link_name).translation();
                current_state.getJacobian(joint_model_group, current_state.getLinkModel(link_name), reference_point, J_i_dynamic_size);

            
                J_i = J_i_dynamic_size;
                J_i_trans = J_i.block<3, 7>(0, 0, 3, J_i.cols());

                // Compute the jacobian for the closest point on link i
                // Transform the minimal distance point into the link frame
                Eigen::Vector3d min_distance_pair_first_pos_link_frame = current_state.getGlobalLinkTransform(link_name).inverse() * min_distance_pair_first_pos;
                Eigen::Vector3d min_distance_pair_second_pos_link_frame = current_state.getGlobalLinkTransform(link_name).inverse() * min_distance_pair_second_pos;
                Eigen::Vector3d repulsion_direction_link_frame = min_distance_pair_first_pos_link_frame - min_distance_pair_second_pos_link_frame;
                //print the position of the closest point
                //RCLCPP_INFO(node->get_logger(), "Closest point position: %f, %f, %f", min_distance_pair_first_pos_link_frame.x(), min_distance_pair_first_pos_link_frame.y(), min_distance_pair_first_pos_link_frame.z());
                // Compute the Jacobian for the closest point on the link i and store it under J_closest
                Eigen::MatrixXd J_dynamic;
                Eigen::Matrix<double, 6, 7> J_closest;
                // Compute the Jacobian for the closest point on the link i with J_closest
                current_state.getJacobian(joint_model_group, current_state.getLinkModel(link_name), min_distance_pair_first_pos_link_frame, J_dynamic);
                
                J_closest = J_dynamic;
                //reshape into 42 array in column major order

                // Create a flat 42-element Eigen vector.
                Eigen::Matrix<double, 42, 1> J_closest_vec = Eigen::Map<Eigen::Matrix<double, 42, 1>>(J_closest.data(), J_closest.size());
                // Convert the Eigen vector into a flat std::vector<double>
                std::vector<double> flat_jacobian(J_closest_vec.data(), J_closest_vec.data() + J_closest_vec.size());

                // Save repulsion direction values.
                if (link_name == "fr3_link2")
                {
                    link2x.push_back(repulsion_direction.x());
                    link2y.push_back(repulsion_direction.y());
                    link2z.push_back(repulsion_direction.z());
                    // Append the 42 numbers to the flat container.
                    jacobian_array2.insert(jacobian_array2.end(), flat_jacobian.begin(), flat_jacobian.end());
                }
                else if (link_name == "fr3_link3")
                {
                    link3x.push_back(repulsion_direction.x());
                    link3y.push_back(repulsion_direction.y());
                    link3z.push_back(repulsion_direction.z());
                    jacobian_array3.insert(jacobian_array3.end(), flat_jacobian.begin(), flat_jacobian.end());
                }
                else if (link_name == "fr3_link4")
                {
                    link4x.push_back(repulsion_direction.x());
                    link4y.push_back(repulsion_direction.y());
                    link4z.push_back(repulsion_direction.z());
                    jacobian_array4.insert(jacobian_array4.end(), flat_jacobian.begin(), flat_jacobian.end());
                }
                else if (link_name == "fr3_link5")
                {
                    link5x.push_back(repulsion_direction.x());
                    link5y.push_back(repulsion_direction.y());
                    link5z.push_back(repulsion_direction.z());
                    jacobian_array5.insert(jacobian_array5.end(), flat_jacobian.begin(), flat_jacobian.end());
                }
                else if (link_name == "fr3_link6")
                {
                    link6x.push_back(repulsion_direction.x());
                    link6y.push_back(repulsion_direction.y());
                    link6z.push_back(repulsion_direction.z());
                    jacobian_array6.insert(jacobian_array6.end(), flat_jacobian.begin(), flat_jacobian.end());
                }
                else if (link_name == "fr3_link7")
                {
                    link7x.push_back(repulsion_direction.x());
                    link7y.push_back(repulsion_direction.y());
                    link7z.push_back(repulsion_direction.z());
                    jacobian_array7.insert(jacobian_array7.end(), flat_jacobian.begin(), flat_jacobian.end());
                }
                else if (link_name == "fr3_hand")
                {
                    handx.push_back(repulsion_direction.x());
                    handy.push_back(repulsion_direction.y());
                    handz.push_back(repulsion_direction.z());
                    jacobian_array_hand.insert(jacobian_array_hand.end(), flat_jacobian.begin(), flat_jacobian.end());
                }
                else if (link_name == "fr3_leftfinger")
                {
                    hand_tcp_x.push_back(repulsion_direction.x());
                    hand_tcp_y.push_back(repulsion_direction.y());
                    hand_tcp_z.push_back(repulsion_direction.z());
                    jacobian_array_hand_tcp.insert(jacobian_array_hand_tcp.end(), flat_jacobian.begin(), flat_jacobian.end());
                }
                // Visualization for the current link
                // Create a counter that increases for every marker you create.
                static int marker_counter = 0;

                visualization_msgs::msg::Marker spheres;
                spheres.header.frame_id = "base";
                spheres.header.stamp = node->now();
                // Instead of using just link_name, use marker_counter to ensure uniqueness.
                spheres.id = marker_counter++;
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
                arrow.header.frame_id = "base";
                arrow.header.stamp = node->now();
                arrow.id = marker_counter++;  // Unique id for arrow marker
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.scale.x = 0.01; // Shaft diameter
                arrow.scale.y = 0.015; // Head diameter
                arrow.color.r = 1.0;
                arrow.color.g = 0.0;
                arrow.color.b = 0.0;
                arrow.color.a = 1.0;
                arrow.lifetime = rclcpp::Duration(40ms);
                arrow.points.push_back(p1);
                arrow.points.push_back(p2);

                // Append the markers to your MarkerArray.
                mkarray.markers.push_back(spheres);
                mkarray.markers.push_back(arrow);
            }
        }
        // Publish all markers
        rviz_publisher->publish(mkarray);

        // Log all saved repulsion directions
        /*for (const auto &[link, direction] : repulsion_directions_per_link)
        {
            RCLCPP_INFO(node->get_logger(), "Repulsion direction for link %s: (%.4f, %.4f, %.4f)",
                        link.c_str(), direction.x(), direction.y(), direction.z());
            //Publish coordinates of closest point min_distance_pair_first_pos
            RCLCPP_INFO(node->get_logger(), "Closest point position for link %s: (%.4f, %.4f, %.4f)",
                        link.c_str(), min_distance_pair_second_pos.x(), min_distance_pair_second_pos.y(), min_distance_pair_second_pos.z());

            
        }*/

        //publish the closest distance
        messages_fr3::msg::ClosestPoint closest_distance_msg;
        // x, y, z (repulsion direction)
        closest_distance_msg.frameeex = hand_tcp_x;
        closest_distance_msg.frameeey = hand_tcp_y;
        closest_distance_msg.frameeez = hand_tcp_z;
        closest_distance_msg.jacobianee = jacobian_array_hand_tcp;
        
        closest_distance_msg.frame2x = link2x;
        closest_distance_msg.frame2y = link2y;
        closest_distance_msg.frame2z = link2z;
        closest_distance_msg.jacobian2 = jacobian_array2;

        closest_distance_msg.frame3x = link3x;
        closest_distance_msg.frame3y = link3y;
        closest_distance_msg.frame3z = link3z; 
        closest_distance_msg.jacobian3 = jacobian_array3;

        closest_distance_msg.frame4x = link4x;
        closest_distance_msg.frame4y = link4y;
        closest_distance_msg.frame4z = link4z;
        closest_distance_msg.jacobian4 = jacobian_array4;

        closest_distance_msg.frame5x = link5x;
        closest_distance_msg.frame5y = link5y;
        closest_distance_msg.frame5z = link5z;
        closest_distance_msg.jacobian5 = jacobian_array5;

        closest_distance_msg.frame6x = link6x;
        closest_distance_msg.frame6y = link6y;
        closest_distance_msg.frame6z = link6z;
        closest_distance_msg.jacobian6 = jacobian_array6;

        closest_distance_msg.frame7x = link7x;
        closest_distance_msg.frame7y = link7y;
        closest_distance_msg.frame7z = link7z;
        closest_distance_msg.jacobian7 = jacobian_array7;

        closest_distance_msg.framehandx = handx;
        closest_distance_msg.framehandy = handy;
        closest_distance_msg.framehandz = handz;
        closest_distance_msg.jacobianhand = jacobian_array_hand;


        //only publish if there are obstacles
        if (num_objects > 0){
            closest_distance_publisher->publish(closest_distance_msg);
        }
        /*RCLCPP_INFO_STREAM(node->get_logger(), "the closest link is: " << min_distance_pair_first);
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RCLCPP_INFO_STREAM(node->get_logger(), "time taken to run the loop: " << elapsed_time.count() / 1000.0 << " seconds");*/

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
