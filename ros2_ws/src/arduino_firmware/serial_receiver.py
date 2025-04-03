#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import serial

#######################################################################################################################
class serialReceiver(Node):
    def __init__(self):
        super().__init__("serial_receiver")
        self.declare_parameter("port",      "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)

        self.aduino_port_      = self.get_parameter("port").value
        self.aduino_baud_rate_ = self.get_parameter("baud_rate").value
        self.frequency_        = 0.01

        self.publication_ = self.create_publisher(String, "serial_receiver", 10)
        self.timer_       = self.create_timer(self.frequency_, self.timerCallback)
        self.arduinoSerial_ = serial.Serial(port=self.aduino_port_, baudrate=self.aduino_baud_rate_, timeout=0.1)
    def timerCallback(self):
        if (rclpy.ok() == True) and (self.arduinoSerial_.is_open == True):
            intMsg = self.arduinoSerial_.readline()
            try:
                intMsg.decode("utf-8")
            except:
                self.get_logger().info("serial_receiver: intMsg decoding failed") 
                return
            msgObj = String()
            msgObj.data = str(intMsg)
            self.publication_.publish(msgObj)

#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = serialReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
#######################################################################################################################
if __name__ == "__main__": main()


