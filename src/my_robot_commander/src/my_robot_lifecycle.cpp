#include <memory>
#include <thread>
#include <string>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;                  // for using _1
 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//https://github.com/ros2/example_interfaces/tree/rolling/msg
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>    //variable size
#include "my_robot_interface/msg/my_robot_leg1_pose_target.hpp"
#include <example_interfaces/msg/bool.hpp>
using ros_string   = example_interfaces::msg::String;
using ros_array    = example_interfaces::msg::Float64MultiArray;
using custom_array = my_robot_interface::msg::MyRobotLeg1PoseTarget;
using ros_bool     = example_interfaces::msg::Bool;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class MyRobotLifecycleManager : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit MyRobotLifecycleManager(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("my_robot_lifecycle", options) {
        RCLCPP_INFO(get_logger(), "constructor(): %s", current_lifecycle_state_.c_str());

        moveit_node_ = std::make_shared<rclcpp::Node>("moveit_interface_node", this->get_namespace(), options);
        callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        leg1_interface_ = std::make_shared<MoveGroupInterface>(moveit_node_, planning_group_);
        if (!leg1_interface_->startStateMonitor(2.0)) {
            RCLCPP_ERROR(get_logger(), "constructor(): failed to start state monitor");
        }
 
        current_lifecycle_state_ = "state_initialized";
        RCLCPP_INFO(get_logger(), "constructor(): %s", current_lifecycle_state_.c_str());
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_configure(): %s", current_lifecycle_state_.c_str());

        leg1_interface_->setEndEffectorLink(endEffector_link_);
        leg1_interface_->setPlanningTime(5.0);
        
        leg1_load_current_robot_state_();
        RCLCPP_INFO(get_logger(), "on_configure(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                    endEffector_x_, endEffector_y_, endEffector_z_);
        current_lifecycle_state_ = "state_configured";
        RCLCPP_INFO(get_logger(), "on_configure(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_activation(): %s", current_lifecycle_state_.c_str());

        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;
        leg1_named_subscriber_ = create_subscription<example_interfaces::msg::String>("/leg1_set_named", 10,
            std::bind(&MyRobotLifecycleManager::leg1NamedCallback, this, std::placeholders::_1), sub_options);

        current_lifecycle_state_ = "state_stationary";
        RCLCPP_INFO(get_logger(), "on_activation(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_deactivation(): %s", current_lifecycle_state_.c_str());

        leg1_named_subscriber_.reset();
        leg1_interface_->stop();

        current_lifecycle_state_ = "state_stopped";
        RCLCPP_INFO(get_logger(), "on_deactivation(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_cleanup(): %s", current_lifecycle_state_.c_str());

        leg1_interface_.reset();

        current_lifecycle_state_ = "state_configured";
        RCLCPP_INFO(get_logger(), "on_clean(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "on_shutdown(): %s", current_lifecycle_state_.c_str());

        leg1_interface_->stop();

        current_lifecycle_state_ = "state_stopped";
        RCLCPP_INFO(get_logger(), "on_shutdown(): %s", current_lifecycle_state_.c_str());
        return CallbackReturn::SUCCESS;
    }
    rclcpp::Node::SharedPtr get_moveit_node() const { 
        return moveit_node_; 
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
private:
    void leg1NamedCallback(const example_interfaces::msg::String::SharedPtr msg) {
        leg1_load_current_robot_state_();
        RCLCPP_INFO(get_logger(), "leg1NamedCallback(): current end effector (x, y, z) = (%lf, %lf, %lf)",
                    endEffector_x_, endEffector_y_, endEffector_z_);
        leg1_interface_->setNamedTarget(msg->data);
        planAndExecute();
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void planAndExecute() {
        MoveGroupInterface::Plan plan;
        success_ = (leg1_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success_) {
            leg1_interface_->execute(plan);
        } else {
            RCLCPP_WARN(get_logger(), "planAndExecute(): planning failed");
        }
    }
    void leg1_load_current_robot_state_() {
        current_robot_state_ = leg1_interface_->getCurrentState(0.5);     // wait up to 0.5 seconds 
        if (!current_robot_state_) {
            RCLCPP_ERROR(get_logger(), "leg1_load_current_robot_state_(): timeout to fetch current robot state");
        }
        endEffector_pose_ = leg1_interface_->getCurrentPose(endEffector_link_);
        endEffector_x_ = endEffector_pose_.pose.position.x;
        endEffector_y_ = endEffector_pose_.pose.position.y;
        endEffector_z_ = endEffector_pose_.pose.position.z;
        leg1_interface_->setStartStateToCurrentState();
        //leg1_interface_->setStartState(*leg1_interface_->getCurrentState()); // faster, but risk stalling
    }

    rclcpp::Node::SharedPtr moveit_node_;
    std::shared_ptr<MoveGroupInterface> leg1_interface_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr leg1_named_subscriber_;

    std::atomic<bool> success_{false};
    std::string current_lifecycle_state_ = "state_uninitialized";
    std::string planning_group_          = "leg1";
    std::string endEffector_link_        = "calfSphere";
    moveit::core::RobotStatePtr     current_robot_state_;
    geometry_msgs::msg::PoseStamped endEffector_pose_;
    double endEffector_x_ = 0;
    double endEffector_y_ = 0;
    double endEffector_z_ = 0;
    double to_target_dist       = 0;
    double to_target_dist_thres = 0.01;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto lifecycle_manager_node = std::make_shared<MyRobotLifecycleManager>(node_options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(lifecycle_manager_node->get_node_base_interface());
    executor.add_node(lifecycle_manager_node->get_moveit_node());
    
    std::thread executor_thread([&executor]() { executor.spin(); });
    using lifecycle_msgs::msg::Transition;
    using lifecycle_msgs::msg::State;
    lifecycle_manager_node->trigger_transition(rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE));
    while (lifecycle_manager_node->get_current_state().id() != State::PRIMARY_STATE_INACTIVE && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    lifecycle_manager_node->trigger_transition(rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE));
    executor_thread.join();     //join command needs to wait for something

    //executor.spin();

    rclcpp::shutdown();
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

