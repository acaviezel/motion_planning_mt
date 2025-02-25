#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <visualization_msgs/msg/marker.hpp>




class StaticBox
{
public:
    StaticBox(
        rclcpp::Node::SharedPtr node,
        const std::string& box_name,
        const std::array<double, 3>& position,
        const std::array<double, 3>& dimensions,
        const std::array<double, 4>& orientation
    ) : node_(node),
        box_name_(box_name),
        position_(position),
        dimensions_(dimensions),
        orientation_(orientation)
    {
        planning_scene_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
        addBoxToPlanningScene();
    }

private:

    void addBoxToPlanningScene()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = "fr3_link0";  // Set to your world or base link
        collision_object.id = box_name_;

        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = {dimensions_[0], dimensions_[1], dimensions_[2]}; // Box dimensions: x, y, z

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.x = orientation_[0];
        box_pose.orientation.y = orientation_[1];
        box_pose.orientation.z = orientation_[2];
        box_pose.orientation.w = orientation_[3];

        box_pose.position.x = position_[0];
        box_pose.position.y = position_[1];
        box_pose.position.z = position_[2];

        collision_object.primitives.push_back(box);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_.applyCollisionObject(collision_object);

        RCLCPP_INFO(node_->get_logger(), "Added box %s to the planning scene", box_name_.c_str());
    }

    rclcpp::Node::SharedPtr node_;
    std::string box_name_;
    std::array<double, 3> position_;
    std::array<double, 3> dimensions_;
    std::array<double, 4> orientation_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
};


class StaticCylinder
{
public:
    StaticCylinder(
        rclcpp::Node::SharedPtr node,
        const std::string& cylinder_name,
        const std::array<double, 3>& position,
        double height,
        double radius,
        const std::array<double, 4>& orientation
    ) : node_(node),
        cylinder_name_(cylinder_name),
        position_(position),
        height_(height),
        radius_(radius),
        orientation_(orientation)
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
        cylinder_pose.orientation.x = orientation_[0];
        cylinder_pose.orientation.y = orientation_[1];
        cylinder_pose.orientation.z = orientation_[2];
        cylinder_pose.orientation.w = orientation_[3];

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
    std::array<double, 4> orientation_;
    double height_;
    double radius_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("static_shapes_scene");

    // cylinder0 is used for self-collision avoidance. remove it if planner is used.
    auto cylinder0 = std::make_unique<StaticCylinder>(
        node, "cylinder0",
        std::array<double, 3>{-0.03, 0.0, 0.2}, 0.4, 0.14,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );

    // Example Box: Position (X, Y, Z), Dimensions (Width, Depth, Height), Orientation (Quaternion)
    /*auto staticbox1 = std::make_unique<StaticBox>(
        node, "staticbox1",
        std::array<double, 3>{0.0, 0.0, -0.05},   // Position
        std::array<double, 3>{1.5, 1.5, 0.05},   // Dimensions (Width, Depth, Height)
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0} // Orientation (default upright)
    );*/
    
    //random clyinder scene
    /*auto cylinder1 = std::make_unique<StaticCylinder>(
        node, "cylinder1",
        std::array<double, 3>{0.35, -0.25, 0.45}, 0.5, 0.05,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );

    auto cylinder5 = std::make_unique<StaticCylinder>(
        node, "cylinder5",

        std::array<double, 3>{-0.6, 0.3, 0.2}, 0.4, 0.07,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );

    auto cylinder6 = std::make_unique<StaticCylinder>(
        node, "cylinder6",
        std::array<double, 3>{-0.6, 0.3, 0.2}, 0.4, 0.07,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );

    

    auto cylinder8 = std::make_unique<StaticCylinder>(
        node, "cylinder8",
        std::array<double, 3>{0.45, -0.2, 0.3}, 0.4, 0.07,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.707, 0.0, 0.707, 1.0}          // Orientation for z-axis alignment
    );

    //create laying cylinder
    auto cylinder9 = std::make_unique<StaticCylinder>(
        node, "cylinder9",
        std::array<double, 3>{-0.5, 0.3, 0.4}, 0.2, 0.07,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.707, 0.0, 0.707, 0.0}          // Orientation for z-axis alignment
    );
    

    auto cylinder13 = std::make_unique<StaticCylinder>(
        node, "cylinder13",
        std::array<double, 3>{-0.1, 0.4, 0.35}, 0.4, 0.07,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.707, 0.0, 0.0, 0.707}          // Orientation for z-axis alignment
    );

    auto cylinder14 = std::make_unique<StaticCylinder>(
        node, "cylinder14",
        std::array<double, 3>{0.15, 0.45, 0.45}, 0.3, 0.07,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.707, 0.0, 0.0, 0.707}          // Orientation for z-axis alignment
    );

    auto cylinder15 = std::make_unique<StaticCylinder>(
        node, "cylinder15",
        std::array<double, 3>{0.35, 0.0, 0.35}, 0.3, 0.07,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.707, 0.0, 0.0, 0.707}          // Orientation for z-axis alignment
    );
    */

    //Cylinders for the thre cylinder scene
    
    /*auto cylinder1 = std::make_unique<StaticCylinder>(
        node, "cylinder1",
        std::array<double, 3>{0.5, 0.0, 0.5}, 1, 0.1,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );

    auto cylinder2 = std::make_unique<StaticCylinder>(
        node, "cylinder2",
        std::array<double, 3>{0.1, -0.4, 0.5}, 1, 0.1,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );


    auto cylinder3 = std::make_unique<StaticCylinder>(
        node, "cylinder3",
        std::array<double, 3>{0.1, 0.4, 0.5}, 1, 0.1,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );*/
    

    //Cylinders for the two window scene
    
    // add eight cylinders in the form of two windows with four cylinders each in front of the robot ( x = 0.5, y = 0.0, z = 0.3)
    
    auto cylinder1 = std::make_unique<StaticCylinder>(
        node, "cylinder1",
        std::array<double, 3>{0.5, -0.2, 0.45}, 0.5, 0.05,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );

    auto cylinder2 = std::make_unique<StaticCylinder>(
        node, "cylinder2",
        std::array<double, 3>{0.5, 0.2, 0.45}, 0.5, 0.05,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );

    auto cylinder3 = std::make_unique<StaticCylinder>(
        node, "cylinder3",
        std::array<double, 3>{0.5, 0.0, 0.65}, 0.5, 0.05,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.707, 0.0, 0.0, 0.707}          // Orientation for z-axis alignment
    );

    auto cylinder4 = std::make_unique<StaticCylinder>(
        node, "cylinder4",
        std::array<double, 3>{0.5, 0.0, 0.25}, 0.5, 0.05,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.707, 0.0, 0.0, 0.707}          // Orientation for z-axis alignment
    );

    auto cylinder5 = std::make_unique<StaticCylinder>(
        node, "cylinder5",
        std::array<double, 3>{0.6, -0.2, 0.45}, 0.5, 0.05,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );

    auto cylinder6 = std::make_unique<StaticCylinder>(
        node, "cylinder6",
        std::array<double, 3>{0.6, 0.2, 0.45}, 0.5, 0.05,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.0, 0.0, 0.0, 1.0}          // Orientation for z-axis alignment
    );

    auto cylinder7 = std::make_unique<StaticCylinder>(
        node, "cylinder7",
        std::array<double, 3>{0.6, 0.0, 0.65}, 0.5, 0.05,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.707, 0.0, 0.0, 0.707}          // Orientation for z-axis alignment
    );

    auto cylinder8 = std::make_unique<StaticCylinder>(
        node, "cylinder8",
        std::array<double, 3>{0.6, 0.0, 0.25}, 0.5, 0.05,  // Position, Height: 0.5, Radius: 0.05
        std::array<double, 4>{0.707, 0.0, 0.0, 0.707}          // Orientation for z-axis alignment
    );
    
    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

    // ✅ Timer to continuously publish markers so they don't disappear
    auto timer = node->create_wall_timer(std::chrono::seconds(1), [node, marker_pub]() {
        visualization_msgs::msg::Marker start_marker;
        start_marker.header.frame_id = "fr3_link0";
        start_marker.header.stamp = node->now();
        start_marker.ns = "waypoints";
        start_marker.id = 0;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        start_marker.pose.position.x = 0.3;
        start_marker.pose.position.y = 0.0;
        start_marker.pose.position.z = 0.3;
        start_marker.scale.x = 0.05;  // 0.2 cm
        start_marker.scale.y = 0.05;
        start_marker.scale.z = 0.05;
        start_marker.color.r = 1.0;
        start_marker.color.g = 0.0;
        start_marker.color.b = 0.0;
        start_marker.color.a = 1.0;
        start_marker.lifetime = rclcpp::Duration(0, 0); // Ensures the marker stays

        marker_pub->publish(start_marker);
        /*RCLCPP_INFO(node->get_logger(), "📌 Published START marker at (%.2f, %.2f, %.2f)", 
                    start_marker.pose.position.x, start_marker.pose.position.y, start_marker.pose.position.z);*/

        visualization_msgs::msg::Marker end_marker;
        end_marker.header.frame_id = "fr3_link0";
        end_marker.header.stamp = node->now();
        end_marker.ns = "waypoints";
        end_marker.id = 1;
        end_marker.type = visualization_msgs::msg::Marker::SPHERE;
        end_marker.action = visualization_msgs::msg::Marker::ADD;
        end_marker.pose.position.x = 0.7;
        end_marker.pose.position.y = 0.05;
        end_marker.pose.position.z = 0.4;
        end_marker.scale.x = 0.05;  // 0.2 cm
        end_marker.scale.y = 0.05;
        end_marker.scale.z = 0.05;
        end_marker.color.r = 1.0;
        end_marker.color.g = 0.0;
        end_marker.color.b = 0.0;
        end_marker.color.a = 1.0;
        end_marker.lifetime = rclcpp::Duration(0, 0); // Ensures the marker stays

        marker_pub->publish(end_marker);
        /*RCLCPP_INFO(node->get_logger(), "📌 Published END marker at (%.2f, %.2f, %.2f)", 
                    end_marker.pose.position.x, end_marker.pose.position.y, end_marker.pose.position.z);*/
    });

    RCLCPP_INFO(node->get_logger(), "Created static shapes and markers successfully.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
