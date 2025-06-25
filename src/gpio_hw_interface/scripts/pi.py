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
        
        self.pub = self.create_publisher(DynamicInterfaceGroupValues,
            "/gpio_controller/commands",
            10)
        
        self.create_timer(1.0, self.pub_gpio)
        self.counter = 0
        self.get_logger().info("Hello mock gpio")


    def pub_gpio(self):
        self.counter+= 1
        msg = DynamicInterfaceGroupValues()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.interface_groups = ["xxx"]
        val = InterfaceValue()
        val.interface_names = ["vacuum"]
        if self.counter % 2:
            val.values = [1.0]
        else:
            val.values = [0.0]
        msg.interface_values = [val]
        self.get_logger().info("--")

        self.pub.publish(msg)


    def __gpio_status_handler(self, msg:DynamicInterfaceGroupValues):
        # self.get_logger().info("---")
        # print(msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()