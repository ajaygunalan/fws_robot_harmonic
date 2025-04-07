#!/usr/bin/python3
# filepath: /home/ajay/fws_robot_harmonic/src/velocity_pub/scripts/keyboard_to_joy.py
import rclpy
import sys
import tty
import termios
import select
from rclpy.node import Node
from sensor_msgs.msg import Joy

class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.print_instructions()

    def print_instructions(self):
        print("Keyboard to Joy Publisher\n---------------------------")
        print("Movement Controls:\n  w/s: Forward/Backward\n  a/d: Left/Right\n  q/e: Rotate Left/Right")
        print("Mode Controls:\n  1: In-phase (A)\n  2: Opposite phase (LB)\n  3: Pivot turn (RB)\n  0: No mode\n  CTRL+C to quit")

    def get_key(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def timer_callback(self):
        key = self.get_key()
        if key:
            self.process_key(key)
        self.joy_msg.axes[0] = self.linear_y
        self.joy_msg.axes[1] = self.linear_x
        self.joy_msg.axes[3] = self.angular_z
        self.joy_publisher.publish(self.joy_msg)

    def process_key(self, key):
        if key == 'w': self.linear_x = 1.0  # Forward
        elif key == 's': self.linear_x = -1.0  # Backward
        elif key == 'a': self.linear_y = 1.0  # Left
        elif key == 'd': self.linear_y = -1.0  # Right
        elif key == 'q': self.angular_z = 1.0  # Rotate Left
        elif key == 'e': self.angular_z = -1.0  # Rotate Right
        elif key == ' ':  # Stop
            self.linear_x = self.linear_y = self.angular_z = 0.0
        elif key == '1': self.joy_msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # A button
        elif key == '2': self.joy_msg.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]  # LB
        elif key == '3': self.joy_msg.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]  # RB
        elif key == '0': self.joy_msg.buttons = [0]*11  # Reset modes
        elif key in ('\x03', '\x1b'): raise KeyboardInterrupt  # Ctrl+C/ESC

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    try: rclpy.spin(node)
    except KeyboardInterrupt: print("Keyboard controller stopped")
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
