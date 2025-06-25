#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from rclpy.qos import qos_profile_system_default
from rclpy.clock import Clock

class MyNode(Node):
    def __init__(self):
        node_name="mock_gpio"
        super().__init__(node_name)
        self.sub = self.create_subscription(DynamicInterfaceGroupValues,
            "/gpio_controller/gpio_states",
            self.__gpio_status_handler,
            qos_profile=qos_profile_system_default
            )
        

        self.get_logger().info("Hello mock gpio")



    def __gpio_status_handler(self, msg:DynamicInterfaceGroupValues):
        # self.get_logger().info("---")
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()