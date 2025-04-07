#!/usr/bin/python3
# filepath: /home/ajay/fws_robot_harmonic/src/velocity_pub/scripts/keyboard_to_joy.py
import rclpy
import sys
import tty
import termios
import select
import threading
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy

class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)
        
        # Initialize Joy message
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
        self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  
        
        # Control values
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # Create a timer to publish at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.print_instructions()

    def print_instructions(self):
        print("Keyboard to Joy Publisher")
        print("---------------------------")
        print("Movement Controls:")
        print("  w/s: Forward/Backward")
        print("  a/d: Left/Right")
        print("  q/e: Rotate Left/Right")
        print("Mode Controls:")
        print("  1: In-phase mode (A button)")
        print("  2: Opposite phase mode (LB button)")
        print("  3: Pivot turn mode (RB button)")
        print("  0: No mode")
        print("  CTRL+C to quit")
    
    def get_key(self):
        # Save terminal settings
        settings = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            # Read a key if available, otherwise return None
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                return key
            return None
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    def timer_callback(self):
        key = self.get_key()
        if key:
            self.process_key(key)
            
        # Update axes values based on current control state
        self.joy_msg.axes[0] = self.linear_y  # Left stick left/right
        self.joy_msg.axes[1] = self.linear_x  # Left stick up/down
        self.joy_msg.axes[3] = self.angular_z  # Right stick left/right
        
        # Publish Joy message
        self.joy_publisher.publish(self.joy_msg)
    
    def process_key(self, key):
        # Movement controls
        if key == 'w':
            self.linear_x = 1.0
            print("Forward")
        elif key == 's':
            self.linear_x = -1.0
            print("Backward")
        elif key == 'a':
            self.linear_y = 1.0
            print("Left")
        elif key == 'd':
            self.linear_y = -1.0
            print("Right")
        elif key == 'q':
            self.angular_z = 1.0
            print("Rotate Left")
        elif key == 'e':
            self.angular_z = -1.0
            print("Rotate Right")
        elif key == ' ':
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
            print("Stop")
        
        # Mode controls
        elif key == '1':  # In-phase - A button
            self.joy_msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            print("Mode: In-phase")
        elif key == '2':  # Opposite phase - LB button
            self.joy_msg.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
            print("Mode: Opposite phase")
        elif key == '3':  # Pivot turn - RB button
            self.joy_msg.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
            print("Mode: Pivot turn")
        elif key == '0':  # No mode
            self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            print("Mode: None")
        
        # Quit on Ctrl+C or ESC
        elif key == '\x03' or key == '\x1b':
            raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard controller stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()