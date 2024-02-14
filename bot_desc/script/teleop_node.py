#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # set the speed parameters
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.max_linear_speed = 3.0
        self.max_angular_speed = 0.75
        self.linear_step = 0.5
        self.angular_step = 0.1

        # start the teleoperation loop
        self.teleop_loop()

    def teleop_loop(self):
        print("Teleoperation of the robot (z: forward, s: stop, q: left, d: right, w: backward)")
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while True:
                if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                    key = sys.stdin.read(1)
                    self.process_key(key)

                self.publish_cmd_vel()
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    def process_key(self, key):
        if key == 'z':
            self.linear_speed = min(self.linear_speed + self.linear_step, self.max_linear_speed)
        elif key == 'w':
            self.linear_speed = max(self.linear_speed - self.linear_step, -self.max_linear_speed)
        elif key == 'q':
            self.angular_speed = min(self.angular_speed + self.angular_step, self.max_angular_speed)
        elif key == 'd':
            self.angular_speed = max(self.angular_speed - self.angular_step, -self.max_angular_speed)
        elif key == 's':
            self.linear_speed = 0.0
            self.angular_speed = 0.0

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    rclpy.spin(teleop_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
