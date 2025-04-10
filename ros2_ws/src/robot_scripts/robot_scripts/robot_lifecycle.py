#!/usr/bin/env python3
import time
import threading
import numpy as np
from functools import partial
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from example_interfaces.msg import String

#import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

#######################################################################################################################
class robot_lifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__("robot_lifecycleNode")
        self.get_logger().info(self.get_name()+": initializing")
        self.i2c_, self.pca_ = None, None
        self.pca_frequency_ = 50
        self.timer_ = None
        self.timer_frequency_ = 3    #s

        self.servos_ = []
        self.servoN_ = 12
        self.activated = False
    def on_configure(self, statePre: LifecycleState) -> TransitionCallbackReturn: 
        self.i2c_ = board.I2C()
        self.pca_ = PCA9685(self.i2c_)
        for servoIdx in range(self.servoN_):
            self.servos_.append(servo.Servo(self.pca_.channels[servoIdx]))
        self.timer_ = self.create_timer(self.timer_frequency_, self.timerCallback)
        return TransitionCallbackReturn.SUCCESS
    def on_cleanup(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": cleaning up")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_activate(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": activating")
        self.activated = True
        return super().on_activate(statePre)
    def on_deactivate(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": deactivating")
        self.activated = False
        return super().on_deactivate(statePre)
    def on_shutdown(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": shutting down")
        self.cleanup_()
        return TransitionCallbackReturn.SUCCESS
    def on_error(self, statePre: LifecycleState):
        self.get_logger().info(self.get_name()+": test in __init__ first, otherwise no compilation error")
        self.cleanup_()
        return TransitionCallbackReturn.FAILURE
    def cleanup_(self):
        self.activated = False
        self.pca_.deinit()
        self.i2c_, self.pca_ = None, None
    ###################################################################################################################
    def timerCallback(self):
        if self.activated == False: return
        self.get_logger().info("timerCallback(): move servo 0")

        servoIdx = 0
        for angle in range(180):
            self.servos_[0].angle = angle
            time.sleep(0.03)
        for angle in range(180):
            self.servos_[0].angle = 180 - angle
            time.sleep(0.03)
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = robot_lifecycleNode()
    #node.trigger_configure()
    #node.trigger_activate()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("main(): keyboard interrupt detected. Shutting down the lifecycle.")
    finally:
        node.cleanup_()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


#######################################################################################################################
if __name__ == "__main__": main()





