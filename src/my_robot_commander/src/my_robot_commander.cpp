#include <thread>
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
            reentrant_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            RCLCPP_INFO(node_->get_logger(), "constructor()");

            leg1_interface_ = std::make_shared<MoveGroupInterface>(node_, "leg1");
            leg1_interface_->setMaxVelocityScalingFactor(1.0);
            leg1_interface_->setMaxAccelerationScalingFactor(1.0);
            leg1_interface_->setEndEffectorLink(endEffector_link_);
            leg1_interface_->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0); // world size
            leg1_interface_->setGoalPositionTolerance(0.01); 
            leg1_interface_->setGoalOrientationTolerance(0.1);
            leg1_interface_->setWorkspace(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
            leg1_interface_->setNumPlanningAttempts(10);
            leg1_interface_->setPlanningTime(60.0);                         // 5 seconds max per plan
            while (rclcpp::ok() && !leg1_interface_->startStateMonitor(1.0)) {
                RCLCPP_INFO(node_->get_logger(), "constructor(): waiting for valid MoveGroupInterface state...");
            }
            RCLCPP_INFO(node_->get_logger(), "constructor(): MoveGroupInterface state received and monitor started");

            rclcpp::SubscriptionOptions sub_options;
            sub_options.callback_group = reentrant_group_;
            leg1_named_subscriber_ = node_->create_subscription<ros_string>  ("/leg1_set_named", 10,
                std::bind(&my_robot_commander_class::leg1NamedCallback, this, _1), sub_options);            
            leg1_joint_subscriber_ = node_->create_subscription<ros_array>   ("/leg1_set_joint", 10,
                std::bind(&my_robot_commander_class::leg1JointCallback, this, _1), sub_options);
            leg1_pose_subscriber_  = node_->create_subscription<custom_array>("/leg1_set_pose",  10,
                std::bind(&my_robot_commander_class::leg1PoseCallback,  this, _1), sub_options);
            leg1_walk_subscriber_  = node_->create_subscription<ros_string>  ("/leg1_set_walk",  10,
                std::bind(&my_robot_commander_class::leg1WalkCallback,  this, _1), sub_options);

            leg1_load_current_state_();
            RCLCPP_INFO(node_->get_logger(), "constructor(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                        endEffector_x_, endEffector_y_, endEffector_z_);
        }
        void leg1SetNamedTarget(const std::string &name) {
            leg1_load_current_state_();
            RCLCPP_INFO(node_->get_logger(), "leg1SetNamedTarget(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                        endEffector_x_, endEffector_y_, endEffector_z_);
            leg1_interface_->setNamedTarget(name);
            planAndExecute(leg1_interface_);
        }
        void leg1SetJointTarget(const std::vector<double> &joints) {
            leg1_load_current_state_();
            RCLCPP_INFO(node_->get_logger(), "leg1SetJointTarget(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                        endEffector_x_, endEffector_y_, endEffector_z_);
            leg1_interface_->setJointValueTarget(joints);
            planAndExecute(leg1_interface_);
        }
        void leg1SetPoseTarget(double x, double y, double z, bool use_cartesian_path=false) {
            leg1_load_current_state_();
            RCLCPP_INFO(node_->get_logger(), "leg1SetPoseTarget(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                        endEffector_x_, endEffector_y_, endEffector_z_);

            geometry_msgs::msg::Pose target_pose = endEffector_pose_.pose;
            target_pose.position.x = x;
            target_pose.position.y = y;
            target_pose.position.z = z;
            //leg1_interface_->setGoalOrientationTolerance(std::numbers::pi);
            //target_pose.orientation.w = 1.0; 
            if (use_cartesian_path == false) {
                //success_ = leg1_interface_->setPositionTarget(x, y, z, endEffector_link_);
                success_ = leg1_interface_->setApproximateJointValueTarget(target_pose, endEffector_link_);
                if (success_ == false) {
                    RCLCPP_ERROR(node_->get_logger(), "leg1SetPoseTarget(): failed to find IK solution for target!");
                    return; 
                }
                planAndExecute(leg1_interface_);
            } else {
                moveit_msgs::msg::RobotTrajectory trajectory;
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(endEffector_pose_.pose);
                waypoints.push_back(target_pose);

                double fraction = leg1_interface_->computeCartesianPath(waypoints, cartesianConstraintStepsize_, 
                                                                        trajectory);
                if (fraction > cartesianConstraintFractionThreshold_) {
                    leg1_interface_->execute(trajectory);
                } else {
                     RCLCPP_INFO(node_->get_logger(), "leg1SetPoseTarget(): Cartesian computation fraction of "
                                                      "%lf, lower than the threshold of %lf", 
                                                      fraction, cartesianConstraintFractionThreshold_);
                }
            }
            
            //auto IK_planned_pose = leg1_interface_->getPoseTarget().pose; // removed after execution
            leg1_load_current_state_();
            to_target_dist = std::sqrt(std::pow(endEffector_x_ - x, 2) + 
                                       std::pow(endEffector_y_ - y, 2) +
                                       std::pow(endEffector_z_ - z, 2));
            if (to_target_dist > to_target_dist_thres) {
                RCLCPP_WARN(node_->get_logger(), "leg1SetPoseTarget(): target unreachable (likely hit a joint limit)."
                                                 "Settled %f meters away.", to_target_dist);
            }
            RCLCPP_INFO(node_->get_logger(), "leg1SetPoseTarget(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                        endEffector_x_, endEffector_y_, endEffector_z_);
        }
        void leg1SetWalkTarget(const std::string &name) {
            leg1_load_current_state_();
            RCLCPP_INFO(node_->get_logger(), "leg1SetWalkTarget(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                        endEffector_x_, endEffector_y_, endEffector_z_);

            std::vector<std::vector<double>> step_points = {
                {-0.09, 0.09, 0.135},
                {-0.09, 0.05, 0.08},
                {-0.09, 0.01, 0.135},
            };
            if (name == "walk1") {
                while (rclcpp::ok() && (is_walking_ == true)) {
                    for (auto& pt : step_points) {
                        if (is_walking_ == false) break;
                        leg1SetPoseTarget(pt[0], pt[1], pt[2], false); 
                    }
                }
            } else if (name == "walk2") {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(endEffector_pose_.pose);
                for (int pt_idx = 0; pt_idx <= step_points.size(); pt_idx++) {
                    geometry_msgs::msg::Pose target_pose = endEffector_pose_.pose;
                    waypoints.push_back(target_pose);
                    waypoints[pt_idx+1].position.x = step_points[pt_idx][0];
                    waypoints[pt_idx+1].position.y = step_points[pt_idx][1];
                    waypoints[pt_idx+1].position.z = step_points[pt_idx][2];
                }

                moveit_msgs::msg::RobotTrajectory trajectory;
                while (rclcpp::ok() && (is_walking_ == true)) {
                    double fraction = leg1_interface_->computeCartesianPath(waypoints, cartesianConstraintStepsize_,
                                                                            trajectory);
                    if (fraction > cartesianConstraintFractionThreshold_) {
                        leg1_interface_->execute(trajectory);
                    } else {
                         RCLCPP_INFO(node_->get_logger(), "leg1SetWalkTarget(): Cartesian computation fraction of "
                                                          "%lf, lower than the threshold of %lf",
                                                          fraction, cartesianConstraintFractionThreshold_);
                    }
                }
            } else if (name == "walk3") {
                double z0 = 0.135; 
                double y0 = 0.09;
                double x0 = -0.09;  
                double traj_arc_rad = 0.04; 
                int traj_arc_N      = 10;    
                int traj_lin_N      = 5;   
                std::vector<geometry_msgs::msg::Pose> waypoints;
                for (int arc_idx = 0; arc_idx <= traj_arc_N; arc_idx++) {
                    double arc_angle = M_PI*(arc_idx/traj_arc_N);

                    geometry_msgs::msg::Pose target_pose = endEffector_pose_.pose;
                    target_pose.position.x = x0;
                    target_pose.position.y = y0 - (std::cos(arc_angle) * traj_arc_rad);
                    target_pose.position.z = z0 - (std::sin(arc_angle) * traj_arc_rad); 
            
                    waypoints.push_back(target_pose);
                }
                for (int lin_idx = 1; lin_idx <= traj_lin_N ; lin_idx++) {
                    double lin_frac = (double) lin_idx/traj_lin_N;
           
                    geometry_msgs::msg::Pose target_pose = endEffector_pose_.pose;
                    target_pose.position.x = x0;
                    target_pose.position.y = (y_0 - 2*traj_arc_rad) + lin_frac*2*traj_arc_rad;
                    target_pose.position.z = z0;
            
                    waypoints.push_back(target_pose);           
                }

                moveit_msgs::msg::RobotTrajectory trajectory;
                while (rclcpp::ok() && (is_walking_ == true)) {
                    double fraction = leg1_interface_->computeCartesianPath(waypoints, cartesianConstraintStepsize_,
                                                                            trajectory);
                    if (fraction > cartesianConstraintFractionThreshold_) {
                        leg1_interface_->execute(trajectory);
                    } else {
                         RCLCPP_INFO(node_->get_logger(), "leg1SetWalkTarget(): Cartesian computation fraction of "
                                                          "%lf, lower than the threshold of %lf",
                                                          fraction, cartesianConstraintFractionThreshold_);
                    }
                    leg1_load_current_state_();
                }
            }
        }
    }


        }
    private:
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface) {
            MoveGroupInterface::Plan plan;
            success_ = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success_ == true) {
                interface->execute(plan);
            } else {
                RCLCPP_WARN(node_->get_logger(), "planAndExecute(): planning failed");
            }
        }
        void leg1NamedCallback(const ros_string::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "leg1NamedCallback()");
            leg1SetNamedTarget(msg->data);
        }
        void leg1JointCallback(const ros_array::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "leg1JointCallback()");
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
            RCLCPP_INFO(node_->get_logger(), "leg1PoseCallback()");
            leg1SetPoseTarget(msg->x, msg->y, msg->z, msg->use_cartesian_path);
        }
        void leg1WalkCallback(const ros_string::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "leg1WalkCallback()");
            if (is_walking_ == false) {
                RCLCPP_INFO(node_->get_logger(), "leg1WalkCallback(): starting walking gait loop");
                is_walking_ = true;
                gait_thread_ = std::thread(&my_robot_commander_class::leg1SetWalkTarget, this, msg->data);
            } else {
                RCLCPP_INFO(node_->get_logger(), "leg1WalkCallback(): stopping walking gait loop");
                is_walking_ = false;
                if (gait_thread_.joinable() == true) {
                    gait_thread_.join();
                }
            }
        }
        ////////////////////////////////////////////////////
        void leg1_load_current_state_() {
            //if (!leg1_interface_->getCurrentState(0.5)) {           //wait up to 0.5 seconds 
            //    RCLCPP_ERROR(node_->get_logger(), "Failed to fetch current robot state (timeout)");
            //    return;
            //}
            endEffector_pose_ = leg1_interface_->getCurrentPose(endEffector_link_);
            endEffector_x_ = endEffector_pose_.pose.position.x;
            endEffector_y_ = endEffector_pose_.pose.position.y;
            endEffector_z_ = endEffector_pose_.pose.position.z;
            leg1_interface_->setStartStateToCurrentState();
            //leg1_interface_->setStartState(*leg1_interface_->getCurrentState()); //faster, but risk stalling
        }
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
        std::shared_ptr<MoveGroupInterface> leg1_interface_;
        double cartesianConstraintStepsize_          = 0.0005;     //meter
        double cartesianConstraintFractionThreshold_ = 1.0;

        std::string endEffector_link_ = "calfSphere";
        geometry_msgs::msg::PoseStamped endEffector_pose_;
        double endEffector_x_ = 0;
        double endEffector_y_ = 0;
        double endEffector_z_ = 0;
        bool   success_             = false;        
        double to_target_dist       = 0;
        double to_target_dist_thres = 0.01;

        bool is_walking_ = false;
        std::thread gait_thread_;
        
        rclcpp::Subscription<ros_string>  ::SharedPtr leg1_named_subscriber_;      
        rclcpp::Subscription<ros_array>   ::SharedPtr leg1_joint_subscriber_;
        rclcpp::Subscription<custom_array>::SharedPtr leg1_pose_subscriber_;
        rclcpp::Subscription<ros_string>  ::SharedPtr leg1_walk_subscriber_;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<rclcpp::Node>("my_robot_commander");
    executor.add_node(node);
    std::thread spin_thread([&executor]() {
        executor.spin();
    });
    auto commander = std::make_shared<my_robot_commander_class>(node);
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



