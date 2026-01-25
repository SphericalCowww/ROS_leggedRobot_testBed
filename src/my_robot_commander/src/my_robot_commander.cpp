#include <thread>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
//https://github.com/ros2/example_interfaces/tree/rolling/msg
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>    //variable size
#include "my_robot_interface/msg/my_robot_leg1_pose_target.hpp"
#include <example_interfaces/msg/bool.hpp>

#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_response.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using MoveGroupSequence  = moveit_msgs::action::MoveGroupSequence;
using ExecuteTrajectory  = moveit_msgs::action::ExecuteTrajectory;
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

            leg1_interface_ = std::make_shared<MoveGroupInterface>(node_, planning_group_);
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
            //leg1_interface_->setGoalOrientationTolerance(M_PI);
            //target_pose.orientation.w = 1.0; 
            if (use_cartesian_path == false) {
                //success_ = leg1_interface_->setPositionTarget(x, y, z, endEffector_link_);
                success_ = leg1_interface_->setApproximateJointValueTarget(target_pose, endEffector_link_);
                if (success_ == false) {
                    RCLCPP_ERROR(node_->get_logger(), "leg1SetPoseTarget(): failed to find IK solution for target!");
                }
                planAndExecute(leg1_interface_);
            } else {
                moveit_msgs::msg::RobotTrajectory trajectory;
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(endEffector_pose_.pose);
                waypoints.push_back(target_pose);

                double fraction = leg1_interface_->computeCartesianPath(waypoints, cartesianConstraintStepsize_, 
                                                                        trajectory);
                if (cartesianConstraintFractionThreshold_ <= fraction) {
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
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            if (name == "walk1") {
                while (rclcpp::ok() && (is_walking_ == true)) {
                    for (auto& pt : step_points) {
                        if (is_walking_ == false) break;
                        leg1SetPoseTarget(pt[0], pt[1], pt[2], false); 
                    }
                }
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            } else if (name == "walk2") {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(endEffector_pose_.pose);
                for (int pt_idx = 0; pt_idx <= int(step_points.size()); pt_idx++) {
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
                    if (cartesianConstraintFractionThreshold_ <= fraction) {
                        leg1_interface_->execute(trajectory);
                    } else {
                         RCLCPP_INFO(node_->get_logger(), "leg1SetWalkTarget(): Cartesian computation fraction of "
                                                          "%lf, lower than the threshold of %lf",
                                                          fraction, cartesianConstraintFractionThreshold_);
                    }
                }
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            } else if (name == "walk3") {
                double z0 = 0.135; 
                double y0 = 0.09;
                double x0 = -0.09;  
                double traj_arc_rad = 0.04; 
                int traj_arc_N      = 10;    
                int traj_lin_N      = 5;   
                std::vector<geometry_msgs::msg::Pose> waypoints;
                for (int arc_idx = 0; arc_idx <= traj_arc_N; arc_idx++) {
                    double arc_angle = (M_PI*arc_idx)/traj_arc_N;

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
                    target_pose.position.y = (y0 - 2*traj_arc_rad) + lin_frac*2*traj_arc_rad;
                    target_pose.position.z = z0;
            
                    waypoints.push_back(target_pose);           
                }

                moveit_msgs::msg::RobotTrajectory trajectory;
                while (rclcpp::ok() && (is_walking_ == true)) {
                    double fraction = leg1_interface_->computeCartesianPath(waypoints, cartesianConstraintStepsize_,
                                                                            trajectory);
                    if (cartesianConstraintFractionThreshold_ <= fraction) {
                        leg1_interface_->execute(trajectory);
                    } else {
                         RCLCPP_INFO(node_->get_logger(), "leg1SetWalkTarget(): Cartesian computation fraction of "
                                                          "%lf, lower than the threshold of %lf",
                                                          fraction, cartesianConstraintFractionThreshold_);
                    }
                    leg1_load_current_state_();
                }
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            } else if (name == "walk4") {
                leg1_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
                double x0 = -0.09;  
                double y0 =  0.01;
                double z0 =  0.13; 
                double traj_arc_rad = 0.04; 

                geometry_msgs::msg::Pose pose_start = endEffector_pose_.pose;
                pose_start.position.x = x0; 
                pose_start.position.y = y0; 
                pose_start.position.z = z0;

                geometry_msgs::msg::Pose pose_top = pose_start;
                pose_top.position.y = y0 + traj_arc_rad; 
                pose_top.position.z = z0 - traj_arc_rad;

                geometry_msgs::msg::Pose pose_land = pose_start;
                pose_land.position.y = y0 + 2*traj_arc_rad;

                using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
                auto action_client = rclcpp_action::create_client<MoveGroupSequence>(node_,"/sequence_move_group");
                if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
                   RCLCPP_ERROR(node_->get_logger(), 
                                "leg1SetWalkTarget(): error waiting for action server /sequence_move_group");
                }
                moveit_msgs::msg::RobotState start_state_msg;
                moveit::core::robotStateToRobotStateMsg(*current_state_, start_state_msg);
                
                moveit_msgs::msg::MotionSequenceRequest sequence_request;
                
                moveit_msgs::msg::MotionSequenceItem traj1;
                traj1.req.start_state = start_state_msg;
                traj1.blend_radius    = 0.001;
                traj1.req.planner_id  = "LIN"; 
                traj_wrap_(traj1);
                traj1.req.goal_constraints.push_back(create_pose_constraints(endEffector_link_, pose_land));
                sequence_request.items.push_back(traj1);

                moveit_msgs::msg::MotionSequenceItem traj2;
                traj2.blend_radius   = 0.001;
                traj2.req.planner_id = "PTP";
                traj_wrap_(traj2);
                traj2.req.goal_constraints.push_back(create_pose_constraints(endEffector_link_, pose_top));
                sequence_request.items.push_back(traj2);

                moveit_msgs::msg::MotionSequenceItem traj3;
                traj3.blend_radius   = 0.0;                       //last one must be 0
                traj3.req.planner_id = "PTP";
                traj_wrap_(traj3);
                traj3.req.goal_constraints.push_back(create_pose_constraints(endEffector_link_, pose_start));
                sequence_request.items.push_back(traj3);

                while (rclcpp::ok() && is_walking_) {
                    leg1_load_current_state_();
                    moveit::core::robotStateToRobotStateMsg(*current_state_,
                                                            sequence_request.items[0].req.start_state);
                    auto goal_msg = MoveGroupSequence::Goal();
                    goal_msg.request = sequence_request;
                    goal_msg.planning_options.planning_scene_diff.is_diff             = true;
                    goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;
                    auto goal_handle_future   = action_client->async_send_goal(goal_msg);
                    auto action_result_future = action_client->async_get_result(goal_handle_future.get());
                    std::future_status action_status;
                    do {
                        switch (action_status = action_result_future.wait_for(std::chrono::seconds(1)); 
                                action_status)
                        {
                            case std::future_status::deferred:
                                RCLCPP_ERROR(node_->get_logger(), "leg1SetWalkTarget(): deferred");
                                break;
                            case std::future_status::timeout:
                                RCLCPP_INFO(node_->get_logger(),  "leg1SetWalkTarget(): timeout");
                                break;
                            case std::future_status::ready:
                                RCLCPP_INFO(node_->get_logger(),  "leg1SetWalkTarget(): action ready!");
                                break;
                        }
                    } while (action_status != std::future_status::ready);
                    if (action_result_future.valid()) {
                        auto result = action_result_future.get();
                        RCLCPP_INFO(node_->get_logger(), "leg1SetWalkTarget(): action completed.\n  "
                                                         "Result: %d", static_cast<int>(result.code));
                    } else {
                        RCLCPP_ERROR(node_->get_logger(),  "leg1SetWalkTarget(): action failed");
                    }
                    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            } else if (name == "walk5") {
                double x0 = -0.09;  
                double y0 =  0.01;
                double z0 =  0.13; 
                double traj_arc_rad = 0.04; 

                geometry_msgs::msg::Pose pose_0 = endEffector_pose_.pose;
                pose_0.position.x = x0; 
                pose_0.position.y = y0; 
                pose_0.position.z = z0;

                geometry_msgs::msg::Pose pose_1 = pose_0;
                pose_1.position.y = y0 + 2*traj_arc_rad;

                geometry_msgs::msg::Pose pose_2 = pose_0;
                pose_2.position.y = y0 + traj_arc_rad;
                pose_2.position.z = z0 - traj_arc_rad;
        
                leg1_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
                moveit_msgs::msg::MotionSequenceRequest sequence_request;
                moveit_msgs::msg::RobotState start_state_msg;
                moveit::core::robotStateToRobotStateMsg(*current_state_, start_state_msg);
                moveit_msgs::msg::MotionSequenceItem traj1, traj2, traj3;
                traj1.req.start_state = start_state_msg;
                traj1.blend_radius    = 0.0;
                traj1.req.planner_id  = "LIN"; 
                traj_wrap_(traj1);
                traj1.req.goal_constraints.push_back(create_pose_constraints(endEffector_link_, pose_1));
                traj2.blend_radius   = 0.0;
                traj2.req.planner_id = "LIN";
                traj_wrap_(traj2);
                traj2.req.goal_constraints.push_back(create_pose_constraints(endEffector_link_, pose_2));
                traj3.blend_radius   = 0.0;                       //last one must be 0
                traj3.req.planner_id = "LIN";
                traj_wrap_(traj3);
                traj3.req.goal_constraints.push_back(create_pose_constraints(endEffector_link_, pose_0));
                sequence_request.items.push_back(traj1);
                sequence_request.items.push_back(traj2);
                sequence_request.items.push_back(traj3);
                
                auto sequence_action_client = rclcpp_action::create_client<MoveGroupSequence>(node_,
                                                                                              "/sequence_move_group");
                sequence_action_client->wait_for_action_server(std::chrono::seconds(5));
                auto sequence_goal = moveit_msgs::action::MoveGroupSequence::Goal();
                sequence_goal.request = sequence_request;
                sequence_goal.planning_options.plan_only = true;
                auto sequence_goal_future   = sequence_action_client->async_send_goal (sequence_goal);
                auto sequence_goal_handle   = sequence_goal_future.get();
                auto sequence_result_future = sequence_action_client->async_get_result(sequence_goal_handle);
                auto sequence_result_handle = sequence_result_future.get();
                auto sequence_result        = sequence_result_handle.result->response;
                moveit_msgs::msg::RobotTrajectory saved_walking_traj;
                if (sequence_result.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    saved_walking_traj = sequence_result.planned_trajectories[0]; 
                    RCLCPP_INFO(node_->get_logger(), "leg1SetWalkTarget(): trajectory planned");
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "leg1SetWalkTarget(): trajectory planning failed");
                }

                auto exec_action_client = rclcpp_action::create_client<ExecuteTrajectory>(node_, 
                                                                                          "/execute_trajectory");
                exec_action_client->wait_for_action_server(std::chrono::seconds(5));
                auto joint_model_group = leg1_interface_->getRobotModel()->getJointModelGroup(planning_group_);
                std::vector<double> current_positions;
                while (rclcpp::ok() && is_walking_) {
                    auto exec_goal = moveit_msgs::action::ExecuteTrajectory::Goal();
                    exec_goal.trajectory = saved_walking_traj;

                    leg1_load_current_state_(); 
                    current_state_->copyJointGroupPositions(joint_model_group, current_positions);
                    if(!exec_goal.trajectory.joint_trajectory.points.empty()) {
                        exec_goal.trajectory.joint_trajectory.points[0].positions = current_positions;
                    }
                    
                    auto exec_goal_future = exec_action_client->async_send_goal(exec_goal);
                    auto exec_goal_handle = exec_goal_future.get();
                    if (exec_goal_handle) {
                        auto exec_result_future = exec_action_client->async_get_result(exec_goal_handle);
                        auto exec_result_handle = exec_result_future.get();
                        RCLCPP_INFO(node_->get_logger(), "leg1SetWalkTarget(): step completed");
                    }
                    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            } else if (name == "walk6") {
                double x0 = -0.09;
                double y0 =  0.01;
                double z0 =  0.13;
                double traj_arc_rad = 0.04;

                geometry_msgs::msg::Pose pose_0 = endEffector_pose_.pose;
                pose_0.position.x = x0;
                pose_0.position.y = y0;
                pose_0.position.z = z0;

                geometry_msgs::msg::Pose pose_1 = pose_0;
                pose_1.position.y = y0 + 2*traj_arc_rad;

                geometry_msgs::msg::Pose pose_2 = pose_0;
                pose_2.position.y = y0 + traj_arc_rad;
                pose_2.position.z = z0 - traj_arc_rad;

                leg1_load_current_state_();
                auto joint_model_group = current_state_->getJointModelGroup(planning_group_);
                moveit::core::RobotState state_0(*current_state_);
                moveit::core::RobotState state_1(*current_state_);
                moveit::core::RobotState state_2(*current_state_);
                success_ = state_0.setFromIK(joint_model_group, pose_0); 
                if (success_ == false) {
                    RCLCPP_ERROR(node_->get_logger(), "leg1SetWalkTarget(): IK failed for pose_0");
                    return; 
                }
                success_ = state_1.setFromIK(joint_model_group, pose_1); 
                if (success_ == false) {
                    RCLCPP_ERROR(node_->get_logger(), "leg1SetWalkTarget(): IK failed for pose_1");
                    return;
                }
                success_ = state_2.setFromIK(joint_model_group, pose_2);
                if (success_ == false) {
                    RCLCPP_ERROR(node_->get_logger(), "leg1SetWalkTarget(): IK failed for pose_2");
                    return;
                } 

                auto traj = std::make_shared<robot_trajectory::RobotTrajectory>(leg1_interface_->getRobotModel(), 
                                                                                planning_group_);
                traj->addSuffixWayPoint(state_0, 0.0);
                traj->addSuffixWayPoint(state_1, 0.0);
                traj->addSuffixWayPoint(state_2, 0.0);
                traj->addSuffixWayPoint(state_0, 0.0);

                trajectory_processing::TimeOptimalTrajectoryGeneration traj_gen;
                success_ = traj_gen.computeTimeStamps(*traj, 1.0, 0.8); // frac of vel, accel from joint_limits.yaml
                if (success_ == false) {
                    RCLCPP_ERROR(node_->get_logger(), "leg1SetWalkTarget(): traj timing generation failed");
                    return;
                }
                moveit_msgs::msg::RobotTrajectory traj_msg;
                traj->getRobotTrajectoryMsg(traj_msg);
                std::vector<double> current_positions;
                //leg1_load_current_state_();
                while (rclcpp::ok() && is_walking_) {
                    auto exec_goal = moveit_msgs::action::ExecuteTrajectory::Goal();
                    exec_goal.trajectory = traj_msg;

                    leg1_load_current_state_(); 
                    current_state_->copyJointGroupPositions(joint_model_group, current_positions);
                    if(!exec_goal.trajectory.joint_trajectory.points.empty()) {
                        exec_goal.trajectory.joint_trajectory.points[0].positions = current_positions;
                    }

                    auto exec_goal_future = exec_action_client->async_send_goal(exec_goal);
                    auto exec_goal_handle = exec_goal_future.get(); 
                    if (exec_goal_handle) {
                        auto exec_result_future = exec_action_client->async_get_result(exec_goal_handle);
                        auto exec_result_handle = exec_result_future.get();
                        RCLCPP_INFO(node_->get_logger(), "leg1SetWalkTarget(): step completed");
                    }
                }
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
        }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
            if (msg->data.size() != numberOfJoints) {
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
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void leg1_load_current_state_() {
            current_state_ = leg1_interface_->getCurrentState(0.5);     // wait up to 0.5 seconds 
            if (!current_state_) {
                RCLCPP_ERROR(node_->get_logger(), "leg1_load_current_state_(): timeout to fetch current robot state");
            }
            endEffector_pose_ = leg1_interface_->getCurrentPose(endEffector_link_);
            endEffector_x_ = endEffector_pose_.pose.position.x;
            endEffector_y_ = endEffector_pose_.pose.position.y;
            endEffector_z_ = endEffector_pose_.pose.position.z;
            leg1_interface_->setStartStateToCurrentState();
            //leg1_interface_->setStartState(*leg1_interface_->getCurrentState()); // faster, but risk stalling
        }
        void traj_wrap_(moveit_msgs::msg::MotionSequenceItem &traj) {
            traj.req.pipeline_id = "pilz_industrial_motion_planner";
            traj.req.group_name  = planning_group_;
            traj.req.allowed_planning_time           = 5.0;
            traj.req.max_velocity_scaling_factor     = 1.0;
            traj.req.max_acceleration_scaling_factor = 0.1;
            traj.req.workspace_parameters.header.frame_id = leg1_interface_->getPlanningFrame();
            traj.req.workspace_parameters.min_corner.x    = -1.0;
            traj.req.workspace_parameters.min_corner.y    = -1.0;
            traj.req.workspace_parameters.min_corner.z    = -1.0;
            traj.req.workspace_parameters.max_corner.x    = 1.0;
            traj.req.workspace_parameters.max_corner.y    = 1.0;
            traj.req.workspace_parameters.max_corner.z    = 1.0;
        }
        moveit_msgs::msg::Constraints create_pose_constraints(const std::string& link_name, 
                                                              const geometry_msgs::msg::Pose& pose) {
            geometry_msgs::msg::PoseStamped stamped_pose;
            stamped_pose.header.frame_id = leg1_interface_->getPlanningFrame();
            stamped_pose.pose = pose;

            double tolerance_pos = 0.001;
            double tolerance_ang = 0.01;

            return kinematic_constraints::constructGoalConstraints(link_name, stamped_pose, tolerance_pos, 
                                                                   tolerance_ang);
        }
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::CallbackGroup::SharedPtr reentrant_group_;
        std::shared_ptr<MoveGroupInterface> leg1_interface_;
        double cartesianConstraintStepsize_          = 0.001;     // meter
        double cartesianConstraintFractionThreshold_ = 1.0;

        std::string planning_group_   = "leg1";
        std::string endEffector_link_ = "calfSphere";
        moveit::core::RobotStatePtr     current_state_;
        geometry_msgs::msg::PoseStamped endEffector_pose_;
        double endEffector_x_ = 0;
        double endEffector_y_ = 0;
        double endEffector_z_ = 0;
        std::atomic<bool> success_{   false};
        double to_target_dist       = 0;
        double to_target_dist_thres = 0.01;

        std::atomic<bool> is_walking_{false};
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



