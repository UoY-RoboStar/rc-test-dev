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

import rospy
import time

from sample.msg import *
#from rospy_message_converter import message_converter

def callback_input(data):
	rospy.loginfo("Called on 'input' with value: %d", data.value)

	in_msg = InputAccepted()
	in_msg.stamp = rospy.get_rostime()
	rospy.loginfo("Input accepted")
	input_accepted_pub.publish(in_msg)

	output = Output()
	output.stamp = rospy.get_rostime()
	
	if (data.value > 5):
		output.value = 0
	else:
		output.value = 1
	     
    # Publish output
	output_pub.publish(output)

def main():
        global output_pub, input_accepted_pub

        # Publishers
        input_accepted_pub = rospy.Publisher('/sample/input_accepted', InputAccepted, queue_size=1)
        output_pub = rospy.Publisher('/sample/output', Output, queue_size=1)

        # Subscribers
        rospy.Subscriber('/sample/input', Input, callback_input, queue_size=1)

        # Initialize node
        rospy.init_node('sample', anonymous=True)
        rospy.loginfo("Initialized node 'sample'")
        rospy.spin()

if __name__ == '__main__':
        try:
                main()
        except rospy.ROSInterruptException:
                pass

