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
import time
import smbus

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
        self.speed_accumulation = 0
        self.speed_updates = 0
        self.collision_counter = 0 # Collision counter
        self.victim_counter = 0 # Victim counter i forhold til RGB.

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
        ** Initialise RGB sensor // Hjemme lavet:-)
        ************************************************************"""
        self.bus = smbus.SMBus(1) # I2C bus.
        self.bus.write_byte_data(0x44, 0x01, 0x05)
        time.sleep(1)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)
        
        self.rgb_timer = self.create_timer( # RGB timer, i forhold til aflæsning.
            2.0,  # unit: s
            self.read_rgb_sensor)

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
 
    def get_average_speed(self):
        return self.speed_accumulation/self.speed_updates
    
    def stop_robot(self):
        # Stop robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot stopped.')

    def get_collision_counter(self):
        return self.collision_counter

    def read_rgb_sensor(self):
        try:
            data = self.bus.read_i2c_block_data(0x44, 0x09, 6)  # Read RGB data
            green = data[1] + data[0] / 256
            red = data[3] + data[2] / 256
            blue = data[5] + data[4] / 256

            # Determine the color:
            if red > green and red > blue:
                self.victim_counter += 1
                self.get_logger().info('Victim detected')

        except Exception as e:
            self.get_logger().error(f"Error reading RGB sensor: {e}")

    def get_victim_counter(self):
        return self.victim_counter

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
        collision_distance = 0.15  # Kollisionsafstand

        # Navigation baseret på afstande
        if obstacle_distance_front < stop_distance:
            # Forhindring for tæt på, bevæg baglæns
            self.get_logger().info('Obstacle detected in FRONT. Moving backward.')
            twist.linear.x = -self.linear_velocity
            twist.angular.z = 0.0
            # self.get_logger().info(f"Collision count incremented: {self.collision_counter}")
        elif obstacle_distance_left_front < safety_distance:
            # Forhindring tæt på venstre front, drej til venstre
            # self.get_logger().info('Obstacle detected in FRONT-LEFT. Turning sharply left.')
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity * 1.3
        elif obstacle_distance_right_front < safety_distance:
            # Forhindring tæt på højre front, drej til højre
            # self.get_logger().info('Obstacle detected in FRONT-RIGHT. Turning sharply right.')
            twist.linear.x = self.linear_velocity
            twist.angular.z = -self.angular_velocity * 1.3
        elif obstacle_distance_left < safety_distance:
            # Forhindring tæt på venstre, drej til venstre
            # self.get_logger().info('Obstacle detected in LEFT. Turning left.')
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity * 0.6
        elif obstacle_distance_right < safety_distance:
            # Forhindring tæt på højre, drej til højre
            # self.get_logger().info('Obstacle detected in RIGHT. Turning right.')
            twist.linear.x = self.linear_velocity
            twist.angular.z = -self.angular_velocity * 0.6
        else:
            # Ingen forhindringer tæt på, bevæg fremad
            # self.get_logger().info('No obstacles detected. Moving forward.')
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

        # Calculate average speed:
        self.speed_updates = self.speed_updates + 1
        self.speed_accumulation = self.speed_accumulation + twist.linear.x

        # Calculate collision counter:
        if obstacle_distance_front or obstacle_distance_left_front or obstacle_distance_right_front < collision_distance:
            self.collision_counter += 1
            # self.get_logger().info(f"Collision count incremented: {self.collision_counter}")