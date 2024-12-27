#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

class StaticCylinder
{
public:
    StaticCylinder(
        rclcpp::Node::SharedPtr node,
        const std::string& cylinder_name,
        const std::array<double, 3>& position,
        double height,
        double radius
    ) : node_(node),
        cylinder_name_(cylinder_name),
        position_(position),
        height_(height),
        radius_(radius)
    {
        planning_scene_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
        addCylinderToPlanningScene();
    }

private:
    void addCylinderToPlanningScene()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "fr3_link0";
        collision_object.id = cylinder_name_;

        shape_msgs::msg::SolidPrimitive cylinder;
        cylinder.type = cylinder.CYLINDER;
        cylinder.dimensions = {height_, radius_}; // Cylinder dimensions: height and radius

        geometry_msgs::msg::Pose cylinder_pose;
        cylinder_pose.orientation.w = 1.0; // No rotation
        cylinder_pose.position.x = position_[0];
        cylinder_pose.position.y = position_[1];
        cylinder_pose.position.z = position_[2];

        collision_object.primitives.push_back(cylinder);
        collision_object.primitive_poses.push_back(cylinder_pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_.applyCollisionObject(collision_object);

        RCLCPP_INFO(node_->get_logger(), "Added cylinder %s to the planning scene", cylinder_name_.c_str());
    }

    rclcpp::Node::SharedPtr node_;
    std::string cylinder_name_;
    std::array<double, 3> position_;
    double height_;
    double radius_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
};

class StaticSphere
{
public:
    StaticSphere(
        rclcpp::Node::SharedPtr node,
        const std::string& sphere_name,
        const std::array<double, 3>& position,
        double radius
    ) : node_(node),
        sphere_name_(sphere_name),
        position_(position),
        radius_(radius)
    {
        planning_scene_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
        addSphereToPlanningScene();
    }

private:
    void addSphereToPlanningScene()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "fr3_link0";
        collision_object.id = sphere_name_;

        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type = sphere.SPHERE;
        sphere.dimensions = {radius_}; // Sphere dimension: radius

        geometry_msgs::msg::Pose sphere_pose;
        sphere_pose.orientation.w = 1.0; // No rotation
        sphere_pose.position.x = position_[0];
        sphere_pose.position.y = position_[1];
        sphere_pose.position.z = position_[2];

        collision_object.primitives.push_back(sphere);
        collision_object.primitive_poses.push_back(sphere_pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_.applyCollisionObject(collision_object);

        RCLCPP_INFO(node_->get_logger(), "Added sphere %s to the planning scene", sphere_name_.c_str());
    }

    rclcpp::Node::SharedPtr node_;
    std::string sphere_name_;
    std::array<double, 3> position_;
    double radius_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("static_shapes_scene");

    // Create a cylinder
    auto cylinder1 = std::make_unique<StaticCylinder>(
        node, "cylinder1",
        std::array<double, 3>{0.5, 0.0, 0.35}, 0.7, 0.1 // Position: [0.4, 0.0, 0.35], Height: 0.7, Radius: 0.1
    );


    // Create a cylinder
    auto cylinder2 = std::make_unique<StaticCylinder>(
        node, "cylinder2",
        std::array<double, 3>{0.0, -0.4, 0.35}, 0.7, 0.1 // Position: [0.4, 0.0, 0.35], Height: 0.7, Radius: 0.1
    );

    // Create a cylinder
    auto cylinder3 = std::make_unique<StaticCylinder>(
        node, "cylinder3",
        std::array<double, 3>{0.0, 0.4, 0.35}, 0.7, 0.1 // Position: [0.4, 0.0, 0.35], Height: 0.7, Radius: 0.1
    );


    // Create a sphere
    /*auto sphere1 = std::make_unique<StaticSphere>(
        node, "sphere1",
        std::array<double, 3>{0.1, 0.0, 0.5}, // Position: [0.2, 0.0, 0.35]
        0.05                                    // Radius: 0.05
    );*/

    RCLCPP_INFO(node->get_logger(), "Created static shapes successfully.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
