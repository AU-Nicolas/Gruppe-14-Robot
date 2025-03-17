#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import math
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan


class Turtlebot3ObstacleDetection(Node):

	def __init__(self):
		super().__init__('turtlebot3_obstacle_detection')

		"""************************************************************
		** Initialise variables
		************************************************************"""
		self.linear_velocity = 0.2 # unit: m/s
		self.angular_velocity = 1.5  # unit: m/s
		self.scan_ranges = []
		self.init_scan_state = False  # To get the initial scan data at the beginning
	
		"""************************************************************
		** Initialise ROS publishers and subscribers
		************************************************************"""
		qos = QoSProfile(depth=10)

		# Initialise publishers
		self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

		# Initialise subscribers
		self.scan_sub = self.create_subscription(
					LaserScan,
					'scan',
					self.scan_callback,
					qos_profile=qos_profile_sensor_data)
		self.cmd_vel_raw_sub = self.create_subscription(
					Twist,
					'cmd_vel_raw',
					self.cmd_vel_raw_callback,
					qos)

		"""************************************************************
		** Initialise timers
		************************************************************"""
		self.update_timer = self.create_timer(
					0.010,  # unit: s
					self.update_callback)

		self.get_logger().info('Turtlebot3 obstacle detection node has been initialised.')

	"""*******************************************************************************
	** Callback functions and relevant functions
	*******************************************************************************"""
	def scan_callback(self, msg):
		self.scan_ranges = self.filter_scan(msg.ranges)
		self.init_scan_state = True
	
	def cmd_vel_raw_callback(self, msg):
		self.linear_velocity = msg.linear.x
		self.angular_velocity = msg.angular.z

	def update_callback(self):
		if self.init_scan_state is True:
					self.detect_obstacle()

	def filter_scan(self, scan_ranges):
		sample_view = len(scan_ranges)
		scan_filter = list(scan_ranges)
		for i in range(sample_view):
			if scan_filter[i] == 0.0 or scan_filter[i] == float('inf') or math.isnan(scan_filter[i]):
				scan_filter[i] = 3.5
		return scan_filter

	def detect_obstacle(self):
		# Define angles for different directions
		front_left_range = 330
		front_right_range = 30
		left_mid_range = 300
		left_range = 240
		right_mid_range = 60
		right_range = 120

		# Find minimum distance to obstacles in different directions
		if len(self.scan_ranges) > 0:
			obstacle_distance_front = min(
				min(self.scan_ranges[front_left_range:360]) if len(self.scan_ranges[front_left_range:360]) > 0 else float('inf'),
				min(self.scan_ranges[0:front_right_range]) if len(self.scan_ranges[0:front_right_range]) > 0 else float('inf')
			)
			obstacle_distance_left_mid = min(self.scan_ranges[left_mid_range:350]) if len(self.scan_ranges[left_mid_range:350]) > 0 else float('inf')
			obstacle_distance_left = min(self.scan_ranges[left_range:left_mid_range]) if len(self.scan_ranges[left_range:left_mid_range]) > 0 else float('inf')
			obstacle_distance_right_mid = min(self.scan_ranges[10:right_mid_range]) if len(self.scan_ranges[10:right_mid_range]) > 0 else float('inf')
			obstacle_distance_right = min(self.scan_ranges[right_mid_range:right_range]) if len(self.scan_ranges[right_mid_range:right_range]) > 0 else float('inf')
		else:
			obstacle_distance_front = float('inf')
			obstacle_distance_left_mid = float('inf')
			obstacle_distance_left = float('inf')
			obstacle_distance_right_mid = float('inf')
			obstacle_distance_right = float('inf')

		twist = Twist()
		safety_distance = 0.5  # Safety distance (for turning)
		stop_distance = 0.2  # Stop distance in meters (for reversing)

		# Determine movement based on distance to obstacles
		if obstacle_distance_front < stop_distance:
			twist.linear.x = -self.linear_velocity  # Move backward
			twist.angular.z = 0.0
		elif obstacle_distance_left < safety_distance:
			twist.linear.x = self.linear_velocity  # Move forward
			twist.angular.z = -self.angular_velocity  # Turn right
		elif obstacle_distance_left_mid < safety_distance:
			twist.linear.x = self.linear_velocity  # Move forward
			twist.angular.z = -self.angular_velocity  # Turn right
		elif obstacle_distance_right < safety_distance:
			twist.linear.x = self.linear_velocity  # Move forward
			twist.angular.z = self.angular_velocity  # Turn left
		elif obstacle_distance_right_mid < safety_distance:
			twist.linear.x = self.linear_velocity  # Move forward
			twist.angular.z = self.angular_velocity  # Turn left         
		else:
			twist.linear.x = self.linear_velocity  # Move forward
			twist.angular.z = 0.0  # No rotation

		self.cmd_vel_pub.publish(twist)

def main(args=None):
	rclpy.init(args=args)
	turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
	rclpy.spin(turtlebot3_obstacle_detection)

	turtlebot3_obstacle_detection.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
