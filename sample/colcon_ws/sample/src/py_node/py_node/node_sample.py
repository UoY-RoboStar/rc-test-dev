#!/usr/bin/env python3

# Copyright (c) 2024 University of York and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Pedro Ribeiro - initial definition
#

# This is a minimal ROS Node for exercising the test generator.

import rclpy
import time

from rclpy.node import Node
from sample_interface.msg import *

#from sample.msg import *
#from rospy_message_converter import message_converter
class SampleNode(Node):
	def __init__(self):
		super().__init__("py_node")
		
		# Publishers
		self.input_accepted_pub = self.create_publisher( InputAccepted,'/sample_interface/Input_accepted',1)
		self.output_pub = self.create_publisher( Output,'/sample_interface/Output',1)

		# Subscriber
		self.subscription = self.create_subscription( Input,'/sample_interface/Input', self.callback_input,1)
	def callback_input(self, data):
		self.get_logger().info("Called on 'input' with value: %d", data.value)

		in_msg = InputAccepted()
		in_msg.stamp = rclpy.get_rostime()
		self.get_logger().info("Input accepted")
		self.input_accepted_pub.publish(in_msg)

		output = Output()
		output.stamp = rclpy.get_rostime()
		
		if (data.value > 5):
			output.value = 0
		else:
			output.value = 1
		     
	    	# Publish output
		self.output_pub.publish(output)

def main(args=None):
        # global output_pub, input_accepted_pub


        # Initialize node
        rclpy.init(args=args)
        py_node = SampleNode()
        py_node.get_logger().info("Initialized node 'sample'")
        rclpy.spin(py_node)

if __name__ == '__main__':
        try:
                main()
        except rclpy.ROSInterruptException:
                pass

