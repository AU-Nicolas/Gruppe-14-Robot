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
	def detect_obstacle(self):
		# Definer vinkler for forskellige retninger
		front_left_range = 350
		front_right_range = 10
		left_range = 270
		right_range = 90

		# Find mindste afstand til forhindringer i forskellige retninger
		obstacle_distance_front = min(min(self.scan_ranges[front_left_range:360]), min(self.scan_ranges[0:front_right_range]))
		obstacle_distance_left = min(self.scan_ranges[left_range:350])
		obstacle_distance_right = min(self.scan_ranges[10:right_range])
	
		twist = Twist()
		safety_distance = 0.5  # Sikkerhedsafstand i meter
		stop_distance = 0.2  # Stopafstand i meter

		# Bestem bevægelse baseret på afstand til forhindringer
		if obstacle_distance_front < stop_distance:
			twist.linear.x = -self.linear_velocity  # Kør baglæns
			twist.angular.z = 0.0
		elif obstacle_distance_left < safety_distance:
			twist.linear.x = self.linear_velocity  # Kør fremad
			twist.angular.z = -self.angular_velocity  # Drej til højre
		elif obstacle_distance_right < safety_distance:
			twist.linear.x = self.linear_velocity  # Kør fremad
			twist.angular.z = self.angular_velocity  # Drej til venstre         
		else:
			twist.linear.x = self.linear_velocity  # Kør fremad
			twist.angular.z = 0.0  # Ingen rotation


		self.cmd_vel_pub.publish(twist)
	

	def main(args=None):
		rclpy.init(args=args)
		turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
		rclpy.spin(turtlebot3_obstacle_detection)

		turtlebot3_obstacle_detection.destroy_node()
		rclpy.shutdown()

	
	if __name__ == '__main__':
		main()
