        if obstacle_distance_front < stop_distance:
            # Forhindring for tæt på, bevæg baglæns
            self.get_logger().info('Obstacle detected in FRONT. Moving backward.')
            twist.linear.x = -self.linear_velocity
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            time.sleep(1.5)  # Move backward for 1.5 seconds

            # Determine the direction to turn based on the furthest distance
            if obstacle_distance_right_front > obstacle_distance_left_front:
                self.get_logger().info('Turning right after moving backward.')
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_velocity
            else:
                self.get_logger().info('Turning left after moving backward.')
                twist.linear.x = 0.0
                twist.angular.z = self.angular_velocity

            self.cmd_vel_pub.publish(twist)
            time.sleep(1)  # Rotate for 1 second
            twist.linear.x = 0.0
            twist.angular.z = 0.0