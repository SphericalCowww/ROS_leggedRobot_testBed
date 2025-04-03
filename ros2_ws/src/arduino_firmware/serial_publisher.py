#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import serial


#######################################################################################################################
class serialTransmitter(Node):
    def __init__(self):
        super().__init__("serial_transmitter")
        self.declare_parameter("port",      "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)

        self.aduino_port_      = self.get_parameter("port").value
        self.aduino_baud_rate_ = self.get_parameter("baud_rate").value

        self.subscription_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)
        self.arduinoSerial_ = serial.Serial(port=self.aduino_port_, baudrate=self.aduino_baud_rate_, timeout=0.1)
    def msgCallback(self, msg):
        self.get_logger().info("serial_transmitter: receiving message published on serial "+str(msg.data)) 
        self.arduinoSerial_.write(msg.data.encode("utf-8"))
#######################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = serialTransmitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
#######################################################################################################################
if __name__ == "__main__": main()


