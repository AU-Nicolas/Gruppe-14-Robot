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
import RPi.GPIO as GPIO  # For LED

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
        self.victim_detected_RGB = False
        self.is_rotating = False # Rotationstilstand i forhold til navigation.
        self.collision_detected = False  # State variabel for kollision
        self.last_collision_time = 0
        self.is_in_collision = False  # State variabel for kollision
        self.led_on_time = None
        self.led_active = False
        
        # Path finder variabler:
        self.tight_space_threshold = 0.30  # Minimum bredde for passage
        self.is_in_tight_space = False
        self.tight_space_velocity = 0.20  # Langsommere hastighed i smalle passager
        self.tight_space_angular = 0.9    # Langsommere drejehastighed

        """************************************************************
        ** Initialise GPIO for LED
        ************************************************************"""
        GPIO.setmode(GPIO.BCM)  # Brug BCM GPIO-numre
        self.GPIO_LED = 17  # LED 3.3 v sat til pin 17.
        GPIO.setup(self.GPIO_LED, GPIO.OUT)  # Konfigurer LED som output

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
            0.2,  # unit: s
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
        if self.led_active and (time.time() - self.led_on_time > 1.0):
            GPIO.output(self.GPIO_LED, False) # Turns off the LED.
            self.led_active = False

        if self.init_scan_state is True:
            self.detect_obstacle()
 
    def get_average_speed(self):
        return self.speed_accumulation/self.speed_updates

    def get_speed_accumulation(self):
        return self.speed_accumulation
    
    def get_speed_updates(self):
        return self.speed_updates
    
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

            current_time = time.time()

            # Determine the color/ if victim is detected:
            if red > green and red > blue:
                if not self.victim_detected_RGB or (current_time - self.last_victim_time > 5.0):
                    self.victim_counter += 1
                    self.get_logger().info('Victim detected')
                    GPIO.output(self.GPIO_LED, True) # Turns on the LED.
                    self.led_on_time = current_time
                    self.led_active = True
                    self.victim_detected_RGB = True
                    self.last_victim_time = current_time
            else:
                self.victim_detected_RGB = False

        # In case we get a weird value
        except Exception as e:
            self.get_logger().error(f"Error reading RGB sensor: {e}")

    def destroy_node(self):
        # Clean up GPIO
        GPIO.cleanup()  # Clean up GPIO settings
        super().destroy_node()

    def get_victim_counter(self):
        return self.victim_counter

    def tight_spaces(self, obstacle_distance_left, obstacle_distance_right, 
                obstacle_distance_front, obstacle_distance_left_front, 
                obstacle_distance_right_front):

        twist = Twist()
        passage_width = obstacle_distance_left_front + obstacle_distance_right_front + obstacle_distance_front
    
        # Find centrum af passagen
        center_offset = obstacle_distance_right_front - obstacle_distance_left_front
    
        # Check om vi er i en smal passage
        if (passage_width < self.tight_space_threshold * 2 and 
            obstacle_distance_front > self.tight_space_threshold):
        
            self.is_in_tight_space = True
            self.get_logger().info('Navigating tight space...')

            twist.linear.x = self.tight_space_velocity
        
            # Juster position i forhold til centrum
            if abs(center_offset) > 0.05:  # Tillad små afvigelser
                # Juster til venstre eller højre for at centrere
                turn_strength = max(min(center_offset, 0.5), -0.5)
                twist.angular.z = -turn_strength * self.tight_space_angular
            else:  # Mere plads til venstre, drej lidt til venstre
                    twist.angular.z = 0.0
            
            return True, twist
        
        self.is_in_tight_space = False
        return False, twist
    

    # *** NAVIGATIONS PROGRAMMET ***

    # LIDAR sensorens syn:

    def compute_dynamic_velocity(self, obs_distance, safety_distance, stop_distance):
        """
        Compute dynamic velocities based on the obstacle distance.
        - For linear velocity: If the obstacle is farther than safety_distance, use full speed.
          Otherwise, scale linearly from 0 at stop_distance to full speed at safety_distance.
        - For angular velocity: Suggest an angular speed that is proportional
          to how much closer the obstacle is compared to safety_distance.
          (The sign should be applied externally based on turning direction.)
        """
        if obs_distance > safety_distance:
            linear_x = self.linear_velocity
            angular_component = 0.0
        else:
            ratio = (obs_distance - stop_distance) / (safety_distance - stop_distance)
            ratio = max(0.0, min(1.0, ratio))
            linear_x = self.linear_velocity * ratio * 1.25
            # The further below safety_distance, the sharper the turn (up to self.angular_velocity)
            angular_component = self.angular_velocity * (1 - ratio) * 1.2
        return linear_x, angular_component

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

        # Navigations distancer:
        twist = Twist()
        safety_distance = 0.33  # Sikkerhedsafstand
        stop_distance = 0.19  # Stopafstand
        collision_distance = 0.17  # Kollisionsafstand (Bruges ikke til navigationen)

        # NAVIGATIONs PARAMETRER:

        is_tight_space, tight_space_cmd = self.tight_spaces(
            obstacle_distance_left,
            obstacle_distance_right,
            obstacle_distance_front,
            obstacle_distance_left_front,
            obstacle_distance_right_front
        )
        
        if is_tight_space:
            self.cmd_vel_pub.publish(tight_space_cmd)
            return        

        if obstacle_distance_front < stop_distance:
            # Forhindring for tæt på, find ny rute.
            self.get_logger().info('Obstacle detected in FRONT.')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
                # Determine the direction to turn based on the furthest distance
            if not self.is_rotating:
                if obstacle_distance_right + obstacle_distance_right_front > obstacle_distance_left + obstacle_distance_left_front:
                    self.get_logger().info('Turning right.')
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_velocity * 1.0
                else:
                    self.get_logger().info('Turning left.')
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_velocity * 1.0
                self.is_rotating = True  # Indikerer at robotten roterer
            else:
                self.is_rotating = False  # Stop rotation

            self.cmd_vel_pub.publish(twist)
            time.sleep(0.8)  # Rotations period.
            
            self.is_rotating = False  # Færdig med rotation.

        # Use dynamic velocity when obstacle is in FRONT-LEFT.
        elif obstacle_distance_left_front < safety_distance:
            # Compute dynamic velocities based on measured distance.
            computed_linear, computed_angular = self.compute_dynamic_velocity(obstacle_distance_left_front,
                                                                              safety_distance,
                                                                              stop_distance)
            # Turning left so angular stays positive.
            twist.linear.x = computed_linear
            twist.angular.z = computed_angular
        # Use dynamic velocity when obstacle is in FRONT-RIGHT.
        elif obstacle_distance_right_front < safety_distance:
            computed_linear, computed_angular = self.compute_dynamic_velocity(obstacle_distance_right_front,
                                                                              safety_distance,
                                                                              stop_distance)
            # Turning right so angular becomes negative.
            twist.linear.x = computed_linear
            twist.angular.z = -computed_angular
        elif obstacle_distance_left < safety_distance:
            twist.linear.x = self.linear_velocity * 0.9
            twist.angular.z = self.angular_velocity * 0.7
        elif obstacle_distance_right < safety_distance:
            twist.linear.x = self.linear_velocity * 0.9
            twist.angular.z = -self.angular_velocity * 0.7
        else:
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

        # Calculate average speed:
        self.speed_updates = self.speed_updates + 1
        self.speed_accumulation = self.speed_accumulation + twist.linear.x

        # Calculate collision counter:
        current_time = time.time()

# Check for collision conditions
        is_collision = (obstacle_distance_front < collision_distance or 
                    obstacle_distance_left_front < collision_distance or 
                    obstacle_distance_right_front < collision_distance)
        
        current_time = time.time()
    

        # Hvis vi registrerer en kollision og ikke er i en kollisionstilstand
        if is_collision:
            if not self.is_in_collision:
            # Tjek om der er gået nok tid siden sidste kollision
                if current_time - self.last_collision_time > 1.5:
                    self.collision_counter += 1
                    self.get_logger().info(f"Collision detected! Total collisions: {self.collision_counter}")
                    self.last_collision_time = current_time
                self.is_in_collision = True
        else:
            self.is_in_collision = False