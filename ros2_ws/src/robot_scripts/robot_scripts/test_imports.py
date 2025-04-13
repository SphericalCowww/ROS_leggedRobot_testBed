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

import sys
import pandas as pd
import board
#######################################################################################################################
def main(args=None):
    print(sys.executable)
    df = pd.DataFrame([])
#######################################################################################################################
if __name__ == "__main__": main()





