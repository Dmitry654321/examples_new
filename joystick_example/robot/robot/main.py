#!/usr/bin/python3
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA, Header
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from rclpy.time import Duration


class RobotController(Node):

    def __init__(self):
        self.vehicle_name = os.getenv("VEHICLE_NAME")
        self.user = os.getenv("USER_NAME")

        super().__init__('controller')


        self.subscription = self.create_subscription(
            String,
            f'/{self.user}/{self.vehicle_name}/command',
            self.command_callback,
            10
        )
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)
        self.led_pub = self.create_publisher(LEDPattern, f'{self.vehicle_name}/led_pattern', 10)

        self.get_logger().info("The duckiebot controller initialized and waiting for commands...")

    def change_led(self, color):
        msg = LEDPattern()
        rgb_vals = [ColorRGBA(**color) for _ in range(5)]
        msg.rgb_vals = rgb_vals
        self.get_logger().info(f'Changed color on duckiebot: {rgb_vals[0]}')
        self.led_pub.publish(msg)

    def command_callback(self, msg):
        command = msg.data.lower()
        if command == 'f':
            self.move_forward()
        elif command == 'b':
            self.move_backward()
        elif command == 'l':
            self.turn_left()
        elif command == 'r':
            self.turn_right()
        elif command == 's':
            self.stop_movement()
        elif command == 'gl':
            color = dict(r=0.0, g=1.0, b=0.0, a=0.5)
            self.switch_lights(color)
        elif command == 'bl':
            color = dict(r=0.0, g=0.0, b=1.0, a=0.5)
            self.switch_lights(color)
        elif command == 'rl':
            color = dict(r=1.0, g=0.0, b=0.0, a=0.5)
            self.switch_lights(color)
        elif command == 'sol':
            color = dict(r=1.0, g=1.0, b=1.0, a=0.5)
            self.switch_lights(color)
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def switch_lights(self, color):
        self.get_logger().info("Switching lights")
        self.get_logger().info(f"Color: {color}")
        self.change_led(color)

    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)

    def move_forward(self):
        self.get_logger().info("Moving forward")
        self.run_wheels('forward_callback', 0.5, 0.5)

    def move_backward(self):
        self.get_logger().info("Moving backward")
        self.run_wheels('backward_callback', -1.0, -1.0)

    def turn_left(self):
        self.get_logger().info("Turning left")
        self.run_wheels('right_callback', 0.0, 0.5)
        self.get_clock().sleep_for(Duration(seconds=1))
        self.run_wheels('stop_callback', 0.0, 0.0)
        self.run_wheels('forward_callback', 0.5, 0.5)

    def turn_right(self):
        self.get_logger().info("Turning right")
        self.run_wheels('right_callback', 0.5, 0.0)
        self.get_clock().sleep_for(Duration(seconds=1))
        self.run_wheels('stop_callback', 0.0, 0.0)
        self.run_wheels('forward_callback', 0.5, 0.5)

    def stop_movement(self):
        self.get_logger().info("Stopping movement")
        self.run_wheels('stop_callback', 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
