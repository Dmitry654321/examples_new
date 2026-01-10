#!/usr/bin/env python3
import os
import sys
import tty
import termios
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DispatcherKeyboard(Node):
    def __init__(self):
        super().__init__('dispatcher_keyboard')

        self.vehicle_name = os.getenv("VEHICLE_NAME")
        self.user = os.getenv("USER_NAME")

        self.publisher = self.create_publisher(
            String,
            f'/{self.user}/{self.vehicle_name}/command',
            10
        )

        self.get_logger().info("Keyboard control ready (WASD / arrows, space=STOP)")

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent command: {command}")


class TTYKeyReader:
    def __init__(self):
        self.fd = os.open("/dev/tty", os.O_RDONLY)
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def close(self):
        if self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            os.close(self.fd)
            self.fd = None

    def read_key(self) -> str:
        ch1 = os.read(self.fd, 1).decode(errors="ignore")
        if ch1 == "\x1b":  # ESC
            ch2 = os.read(self.fd, 1).decode(errors="ignore")
            ch3 = os.read(self.fd, 1).decode(errors="ignore")
            return ch1 + ch2 + ch3
        return ch1


def main(args=None):
    rclpy.init(args=args)
    node = DispatcherKeyboard()
    reader = TTYKeyReader()

    try:
        while rclpy.ok():
            key = reader.read_key()

            if key is None:
                continue

            if key == 'w':
                node.send_command('f')
            elif key in ('s', 'S'):
                node.send_command('b')
            elif key in ('a', 'A'):
                node.send_command('l')
            elif key in ('d', 'D'):
                node.send_command('r')
            elif key == ' ':
                node.send_command('s')
            elif key == 'b':
                node.send_command('bl')
            elif key == 'g':
                node.send_command('gl')
            elif key == 'r':
                node.send_command('rl')
            elif key == 'W':
                node.send_command('sol')
            elif key == '\x03':  # Ctrl+C
                break

    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
