#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
//https://github.com/ros2/example_interfaces/tree/rolling/msg
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>    //variable size
#include "my_robot_interface/msg/ma_robot_arm_pose_target.hpp"
#include <example_interfaces/msg/bool.hpp>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;                  // for using _1
using ros_string   = example_interfaces::msg::String;
using ros_array    = example_interfaces::msg::Float64MultiArray;
using custom_array = my_robot_interface::msg::MaRobotArmPoseTarget;
using ros_bool     = example_interfaces::msg::Bool;

class ma_robot_commander_class
{
    public:
        ma_robot_commander_class(std::shared_ptr<rclcpp::Node> node) {
            node_ = node;
            RCLCPP_INFO(node_->get_logger(), "ma_robot_commander_class::constructor()");

            arm_interface_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            arm_interface_->setMaxVelocityScalingFactor(1.0);
            arm_interface_->setMaxAccelerationScalingFactor(1.0);
            gripper_interface_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

            arm_named_subscriber_ = node_->create_subscription<ros_string> ("arm_set_named", 10,
                std::bind(&ma_robot_commander_class::armNamedCallback, this, _1));            
            arm_joint_subscriber_ = node_->create_subscription<ros_array>  ("arm_set_joint", 10,
                std::bind(&ma_robot_commander_class::armJointCallback, this, _1));
            arm_pose_subscriber_ = node_->create_subscription<custom_array>("arm_set_pose", 10,
                std::bind(&ma_robot_commander_class::armPoseCallback,  this, _1));
            gripper_subscriber_   = node_->create_subscription<ros_bool>   ("gripper_set_open", 10, 
                std::bind(&ma_robot_commander_class::gripperCallback,  this, _1));
        }
        void armSetNamedTarget(const std::string &name) {
            arm_interface_->setStartStateToCurrentState();
            arm_interface_->setNamedTarget(name);
            planAndExecute(arm_interface_);
        }
        void armSetJointTarget(const std::vector<double> &joints) {
            arm_interface_->setStartStateToCurrentState();
            arm_interface_->setJointValueTarget(joints);
            planAndExecute(arm_interface_);
        }
        void armSetPoseTarget(double x, double y, double z, double roll, double pitch, double yaw,
                              bool use_cartesian_path=false) {
            tf2::Quaternion quaternionObj;
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";    

            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            quaternionObj.setRPY(roll, pitch, yaw);   //Euler's angle
            quaternionObj = quaternionObj.normalize();
            target_pose.pose.orientation.x = quaternionObj.getX();
            target_pose.pose.orientation.y = quaternionObj.getY();
            target_pose.pose.orientation.z = quaternionObj.getZ();
            target_pose.pose.orientation.w = quaternionObj.getW();

            arm_interface_->setStartStateToCurrentState();
            if (use_cartesian_path == false) {
                arm_interface_->setPoseTarget(target_pose);
                planAndExecute(arm_interface_);
            } else {
                moveit_msgs::msg::RobotTrajectory trajectory;
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                double fraction = arm_interface_->computeCartesianPath(waypoints, cartesianConstraintStepsize_, trajectory);
                RCLCPP_INFO(node_->get_logger(), "armSetPoseTarget(): cartesianConstraintFraction = %lf", fraction);
                if (fraction == 1) {
                    arm_interface_->execute(trajectory);
                }
            }
        }
        void gripperSetNameTarget(const std::string &name) {
            gripper_interface_->setStartStateToCurrentState();
            gripper_interface_->setNamedTarget(name);
            planAndExecute(gripper_interface_);
        }
        void gripperOpen()  { gripperSetNameTarget("gripper_open"); }
        void gripperClose() { gripperSetNameTarget("gripper_close"); }
    private:
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface) {
            MoveGroupInterface::Plan plan;
            interface->plan(plan);
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success == true) {
                interface->execute(plan);
            }
        }

        void armNamedCallback(const ros_string::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "ma_robot_commander_class::armNamedCallback()");
            armSetNamedTarget(msg->data);
        }
        void armJointCallback(const ros_array::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "ma_robot_commander_class::armJointCallback()");
            msg->layout.dim.resize(1);
            msg->layout.dim[0].size   = msg->data.size();
            msg->layout.dim[0].stride = msg->data.size();
            msg->layout.data_offset = 0;
            if (msg->layout.dim.empty()) {
                RCLCPP_WARN(node_->get_logger(), "armJointCallback(): message empty");
                return;
            }
            else if (msg->layout.dim[0].size != 6) {
                RCLCPP_WARN(node_->get_logger(), "armJointCallback(): incorrect input dimension, expect 6");
                return;
            }
            armSetJointTarget(msg->data);
        }
        void armPoseCallback(const custom_array::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "ma_robot_commander_class::armPoseCallback()");
            armSetPoseTarget(msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw, msg->use_cartesian_path);
        }
        void gripperCallback(const ros_bool::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "ma_robot_commander_class::gripperCallback()");
            if ( msg->data == true) {
                gripperOpen();
            } else {
                gripperClose();
            }
        }
        
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_interface_;
        std::shared_ptr<MoveGroupInterface> gripper_interface_;
        double cartesianConstraintStepsize_ = 0.01;     //meter

        rclcpp::Subscription<ros_string>  ::SharedPtr arm_named_subscriber_;      
        rclcpp::Subscription<ros_array>   ::SharedPtr arm_joint_subscriber_;
        rclcpp::Subscription<custom_array>::SharedPtr arm_pose_subscriber_;
        rclcpp::Subscription<ros_bool>    ::SharedPtr gripper_subscriber_;
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ma_robot_commander");
    auto commander = ma_robot_commander_class(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



