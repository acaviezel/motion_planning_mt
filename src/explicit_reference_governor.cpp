#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;
using Eigen::VectorXd;




std::vector<double> vectorXdToStdVector(const VectorXd& eigen_vec) {
  std::vector<double> std_vec(7);
  for (int i = 0; i < 7; ++i) {
    std_vec[i] = eigen_vec(i);
  }
  return std_vec;
}

Eigen::VectorXd stdVectorToVectorXd(const std::vector<double>& std_vec) {
  Eigen::VectorXd eigen_vec(7);
  for (int i = 0; i < 7; ++i) {
    eigen_vec(i) = std_vec[i];
  }
  return eigen_vec;
}


VectorXd attractionField(const VectorXd q_r, const VectorXd q_v)
{
    VectorXd attraction = q_r - q_v;
    double eta = 0.05; //smoothing parameter
    double distance = (attraction.norm() > eta) ? attraction.norm() : eta;
    attraction = attraction / distance;
    return attraction;
}


VectorXd jointRepulsionField(VectorXd q_v)
{
    // Joint limit: min and max
    VectorXd q_min(7);
    q_min << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    VectorXd q_max(7); 
    q_max << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    
    VectorXd jointRepulsion(7);
    double zeta_q = 0.35; // the influence margin
    double delta_q = 0.15; // the static safety margin
    for (int i = 0; i <7; ++i) {
    jointRepulsion(i) = std::max( (zeta_q - std::abs(q_v(i)-q_min(i))) / (zeta_q-delta_q) , 0.0) - 
                                std::max( (zeta_q - std::abs(q_v(i)-q_max(i))) / (zeta_q - delta_q), 0.0);
    }
    return jointRepulsion;
}




VectorXd navigationField(const VectorXd q_r, const VectorXd q_v)
{
    return attractionField(q_r,q_v) + jointRepulsionField(q_v);
}

double dynamicSafetyMargin()
{
    return 2.0;
}


class ExplicitReferenceGovernor : public rclcpp::Node 
{
    public:
        ExplicitReferenceGovernor() : Node("explicit_reference_governor_node")
        {
            reference_subscription = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                "reference", 10, std::bind(&ExplicitReferenceGovernor::referenceCallback, this, std::placeholders::_1));

            primitive_shape_repulsion_field_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "primitive_shape_repulsion_field", 10, std::bind(&ExplicitReferenceGovernor::primitiveShapeRepulsionFieldCallback, this, std::placeholders::_1));

            applied_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "panda_arm_controller/joint_trajectory", 10);
            
            q_v_old << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785; // initial configuration after launching
            q_v_new << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785; // initial configuration after launching
        }
    private:
        void referenceCallback(const trajectory_msgs::msg::JointTrajectory& msg)
        {
            /*
            trajectory_msgs::msg::JointTrajectory processed_msg = msg;
            VectorXd vec(7);
            vec << 0.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
            std::vector<double> vec_double = convertVectorXdToStdVector(vec);
            processed_msg.points[0].positions = vec_double;
            applied_publisher->publish(processed_msg);
            */
            trajectory_msgs::msg::JointTrajectory processed_msg = msg;
            VectorXd q_r(7);
            q_r = stdVectorToVectorXd(msg.points[0].positions);
            VectorXd q_v_dot = ( navigationField(q_r,q_v_old) + primitiveShapeRepulsionField() ) * dynamicSafetyMargin();
            // VectorXd q_v_dot = ( navigationField(q_r,q_v_old)) * dynamicSafetyMargin();
            double eta = 0.05; //smoothing factor
            double step_size =((q_v_dot*dt).norm() < (q_r - q_v_old).norm()) ? (q_v_dot*dt).norm() : (q_r - q_v_old).norm();
            VectorXd step_direction = q_v_dot;
            double distance = ((q_v_dot).norm() > eta) ? (q_v_dot).norm() : eta;
            step_direction = step_direction / distance;
            q_v_new = q_v_old + step_size * step_direction;
            std::vector<double> std_q_v_new;
            std_q_v_new = vectorXdToStdVector(q_v_new);
            processed_msg.points[0].positions = std_q_v_new;
            applied_publisher->publish(processed_msg);
            q_v_old = q_v_new;
        }

        void primitiveShapeRepulsionFieldCallback(const std_msgs::msg::Float64MultiArray& msg)
        {
            primitve_shape_repulsion_field = msg.data;
        }

        VectorXd primitiveShapeRepulsionField()
        {
            // return the repulsion field that we get from the subscription to distance calculator node
            return stdVectorToVectorXd(primitve_shape_repulsion_field);
        }

        VectorXd q_v_old = VectorXd::Zero(7);
        VectorXd q_v_new = VectorXd::Zero(7);
        double rate = 25;
        double dt = 1/rate;
        std::vector<double> primitve_shape_repulsion_field = {0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr reference_subscription;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr primitive_shape_repulsion_field_subscription;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr applied_publisher;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExplicitReferenceGovernor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}