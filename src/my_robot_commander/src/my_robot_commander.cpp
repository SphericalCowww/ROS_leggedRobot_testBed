#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
//https://github.com/ros2/example_interfaces/tree/rolling/msg
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>    //variable size
#include "my_robot_interface/msg/my_robot_leg1_pose_target.hpp"
#include <example_interfaces/msg/bool.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;                  // for using _1
using ros_string   = example_interfaces::msg::String;
using ros_array    = example_interfaces::msg::Float64MultiArray;
using custom_array = my_robot_interface::msg::MyRobotLeg1PoseTarget;
using ros_bool     = example_interfaces::msg::Bool;

class my_robot_commander_class
{
    public:
        my_robot_commander_class(std::shared_ptr<rclcpp::Node> node) {
            node_ = node;
            RCLCPP_INFO(node_->get_logger(), "my_robot_commander_class::constructor()");

            leg1_interface_ = std::make_shared<MoveGroupInterface>(node_, "leg1");
            leg1_interface_->setMaxVelocityScalingFactor(1.0);
            leg1_interface_->setMaxAccelerationScalingFactor(1.0);
            leg1_interface_->setEndEffectorLink("calfSphere");

            leg1_named_subscriber_ = node_->create_subscription<ros_string> ("leg1_set_named", 10,
                std::bind(&my_robot_commander_class::leg1NamedCallback, this, _1));            
            leg1_joint_subscriber_ = node_->create_subscription<ros_array>  ("leg1_set_joint", 10,
                std::bind(&my_robot_commander_class::leg1JointCallback, this, _1));
            leg1_pose_subscriber_ = node_->create_subscription<custom_array>("leg1_set_pose", 10,
                std::bind(&my_robot_commander_class::leg1PoseCallback,  this, _1));
        }
        void leg1SetNamedTarget(const std::string &name) {
            leg1_interface_->setStartStateToCurrentState();
            leg1_interface_->setNamedTarget(name);
            planAndExecute(leg1_interface_);
        }
        void leg1SetJointTarget(const std::vector<double> &joints) {
            leg1_interface_->setStartStateToCurrentState();
            leg1_interface_->setJointValueTarget(joints);
            planAndExecute(leg1_interface_);
        }
        void leg1SetPoseTarget(double x, double y, double z, bool use_cartesian_path=false) {
            tf2::Quaternion quaternionObj;
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = leg1_interface_->getPlanningFrame(); 

            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = 0.0;
            target_pose.pose.orientation.y = 0.0;
            target_pose.pose.orientation.z = 0.0;
            target_pose.pose.orientation.w = 1.0;

            leg1_interface_->setStartStateToCurrentState();
            if (use_cartesian_path == false) {
                leg1_interface_->setPoseTarget(target_pose);
                planAndExecute(leg1_interface_);
            } else {
                moveit_msgs::msg::RobotTrajectory trajectory;
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                double fraction = leg1_interface_->computeCartesianPath(waypoints,
                                                                        cartesianConstraintStepsize_,
                                                                        trajectory);
                RCLCPP_INFO(node_->get_logger(), 
                            "leg1SetPoseTarget(): cartesianConstraintFraction = %lf", 
                            fraction);
                if (fraction == 1) {
                    leg1_interface_->execute(trajectory);
                }
            }
        }
    private:
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface) {
            MoveGroupInterface::Plan plan;
            //interface->plan(plan);
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success == true) {
                interface->execute(plan);
            } else {
                RCLCPP_WARN(node_->get_logger(), "planAndExecute(): planning failed");
            }
        }

        void leg1NamedCallback(const ros_string::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "my_robot_commander_class::leg1NamedCallback()");
            leg1SetNamedTarget(msg->data);
        }
        void leg1JointCallback(const ros_array::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "my_robot_commander_class::leg1JointCallback()");
            auto numberOfJoints = leg1_interface_->getJointNames().size();
            msg->layout.dim.resize(1);
            msg->layout.dim[0].size   = msg->data.size();
            msg->layout.dim[0].stride = msg->data.size();
            msg->layout.data_offset = 0;
            if (msg->layout.dim.empty()) {
                RCLCPP_WARN(node_->get_logger(), "leg1JointCallback(): message empty");
                return;
            }
            else if (msg->layout.dim[0].size != numberOfJoints) {
                RCLCPP_WARN(node_->get_logger(), "leg1JointCallback(): incorrect input dimension, expect %li",
                            numberOfJoints);
                return;
            }
            leg1SetJointTarget(msg->data);
        }
        void leg1PoseCallback(const custom_array::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "my_robot_commander_class::leg1PoseCallback()");
            leg1SetPoseTarget(msg->x, msg->y, msg->z, msg->use_cartesian_path);
        }
        
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> leg1_interface_;
        double cartesianConstraintStepsize_ = 0.0005;     //meter

        rclcpp::Subscription<ros_string>  ::SharedPtr leg1_named_subscriber_;      
        rclcpp::Subscription<ros_array>   ::SharedPtr leg1_joint_subscriber_;
        rclcpp::Subscription<custom_array>::SharedPtr leg1_pose_subscriber_;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_robot_commander");
    auto commander = my_robot_commander_class(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



