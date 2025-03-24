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
import rclpy


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.linear_velocity = 0.2  # unit: m/s
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
        # Samlede antal scanningsområder fra lidar
        total_ranges = len(self.scan_ranges)

        # Definer sektorer for 180 grader (270 til 90 grader)
        left_start = int(total_ranges * 3 / 4)  # 270 grader
        front_end = int(total_ranges * 1 / 4)  # 90 grader
        half_combined = self.scan_ranges[left_start:] + self.scan_ranges[:front_end]

        # Opdel de 180 grader i 5 lige store sektioner
        section_size = len(half_combined) // 5
        left = half_combined[:section_size]
        left_front = half_combined[section_size:section_size * 2]
        front = half_combined[section_size * 2:section_size * 3]
        right_front = half_combined[section_size * 3:section_size * 4]
        right = half_combined[section_size * 4:]

        # Beregn den mindste afstand til en forhindring i hver sektion
        obstacle_distance_left = min(left) if left else float('inf')
        obstacle_distance_left_front = min(left_front) if left_front else float('inf')
        obstacle_distance_front = min(front) if front else float('inf')
        obstacle_distance_right_front = min(right_front) if right_front else float('inf')
        obstacle_distance_right = min(right) if right else float('inf')

        twist = Twist()
        safety_distance = 0.5  # Sikkerhedsafstand
        stop_distance = 0.2  # Stopafstand

        # Navigation baseret på afstande
        if obstacle_distance_front < stop_distance:
            # Forhindring for tæt på, bevæg baglæns
            self.get_logger().info('Obstacle detected in FRONT. Moving backward.')
            twist.linear.x = -self.linear_velocity
            twist.angular.z = 0.0
        elif obstacle_distance_left_front < safety_distance:
            # Forhindring tæt på venstre front, drej til højre
            self.get_logger().info('Obstacle detected in FRONT-LEFT. Turning sharply right.')
            twist.linear.x = self.linear_velocity
            twist.angular.z = -self.angular_velocity * 1.5
        elif obstacle_distance_right_front < safety_distance:
            # Forhindring tæt på højre front, drej til venstre
            self.get_logger().info('Obstacle detected in FRONT-RIGHT. Turning sharply left.')
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity * 1.5
        elif obstacle_distance_left < safety_distance:
            # Forhindring tæt på venstre, drej skarpt til højre
            self.get_logger().info('Obstacle detected in LEFT. Turning right.')
            twist.linear.x = self.linear_velocity
            twist.angular.z = -self.angular_velocity * 0.8
        elif obstacle_distance_right < safety_distance:
            # Forhindring tæt på højre, drej skarpt til venstre
            self.get_logger().info('Obstacle detected in RIGHT. Turning left.')
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity * 0.8
        else:
            # Ingen forhindringer tæt på, bevæg fremad
            self.get_logger().info('No obstacles detected. Moving forward.')
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