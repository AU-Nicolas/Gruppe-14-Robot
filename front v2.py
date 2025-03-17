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
		self.scan_ranges = msg.ranges
		self.init_scan_state = True
	
	def cmd_vel_raw_callback(self, msg):
		self.linear_velocity = msg.linear.x
		self.angular_velocity = msg.angular.z

	def update_callback(self):
		if self.init_scan_state is True:
					self.detect_obstacle()
	
		# *** NAVIGATIONS PROGRAMMET ***

		# LIDAR sensorens syn:
	def detect_obstacle(self):
		# Samlede antal scanningsområder fra lidar.
		total_ranges = len(self.scan_ranges)
		# TIL FRONT:
		# Definer front for lidar sensoren:
		front_start = int(total_ranges * 3 / 4)
		front_end = int(total_ranges / 4)
		front_combined = self.scan_ranges[front_start:] + self.scan_ranges[:front_end]

		# Opdel de forreste rækkevidder i left, front og right:
		left_range = int(len(front_combined) / 3)
		front_range = int(len(front_combined) / 3)
		right_range = len(front_combined) - left_range - front_range

		# Beregn den mindste afstand til en forhindring i hver "cone"
		obstacle_distance_right = min(front_combined[:left_range]) if left_range > 0 else float('inf')
		obstacle_distance_front = min(front_combined[left_range:left_range + front_range]) if front_range > 0 else float('inf')
		obstacle_distance_left = min(front_combined[left_range + front_range:]) if right_range > 0 else float('inf')

		twist = Twist()
		safety_distance = 0.5
		stop_distance = 0.2

		# Navigationen:
		if obstacle_distance_front < stop_distance:
			# Hvis en forhindring er for tæt foran, bevæg dig baglæns
			twist.linear.x = -self.linear_velocity
			twist.angular.z = 0.0
		elif obstacle_distance_left < safety_distance:
			# Hvis en forhindring er for tæt på venstre side, drej til højre
			twist.linear.x = self.linear_velocity
			twist.angular.z = -self.angular_velocity
		elif obstacle_distance_right < safety_distance:
			# Hvis en forhindring er for tæt på højre side, drej til venstre
			twist.linear.x = self.linear_velocity
			twist.angular.z = self.angular_velocity
		else:
			# Hvis der ikke er nogen forhindringer for tæt på, bevæg dig fremad
			twist.linear.x = self.linear_velocity
			twist.angular.z = 0.0

		self.cmd_vel_pub.publish(twist)
	

	def main(args=None):
		rclpy.init(args=args)
		turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
		rclpy.spin(turtlebot3_obstacle_detection)

		turtlebot3_obstacle_detection.destroy_node()
		rclpy.shutdown()

	
	if __name__ == '__main__':
		main()