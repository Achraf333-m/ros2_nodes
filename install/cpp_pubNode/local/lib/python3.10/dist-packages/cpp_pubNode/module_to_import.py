#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("my_python_node")
        self.get_logger().info("Hello Space Concordia from python node")