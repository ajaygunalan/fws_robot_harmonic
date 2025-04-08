#!/usr/bin/python3
# filepath: /home/ajay/fws_robot_harmonic/src/velocity_pub/scripts/keyboard_to_joy.py
import rclpy
import sys
import tty
import termios
import select
import os
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy

class KBHit:
    def __init__(self):
        # Save the terminal settings
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)
        
        # New terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        self.set_raw_term()
        
    def set_raw_term(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
        
    def set_normal_term(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)
        
    def getch(self):
        return sys.stdin.read(1)
        
    def kbhit(self):
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        return len(dr) > 0

class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        # Publisher setup
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)
        
        # Create Joy message with default values
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
    def update_joy(self, linear_x, linear_y, angular_z, buttons=None):
        # Update Joy message values
        self.joy_msg.axes[0] = linear_y       # Left/Right
        self.joy_msg.axes[1] = linear_x       # Forward/Backward
        self.joy_msg.axes[3] = angular_z      # Rotation
        
        if buttons is not None:
            self.joy_msg.buttons = buttons
            
        # Publish the updated message
        self.joy_publisher.publish(self.joy_msg)

def clear_screen():
    os.system('clear' if os.name == 'posix' else 'cls')

def print_instructions():
    print("╔════════════════════════════════════╗")
    print("║    FWS Robot Keyboard Control      ║")
    print("╚════════════════════════════════════╝")
    print("MOVEMENT CONTROLS:")
    print("  W/S  : Increase/Decrease Forward Speed")
    print("  A/D  : Increase/Decrease Lateral Speed")
    print("  Q/E  : Increase/Decrease Rotation")
    print("  X    : Complete Stop")
    print("MODE CONTROLS:")
    print("  1    : In-phase steering (A button)")
    print("  2    : Opposite-phase steering (LB button)")
    print("  3    : Pivot turn (RB button)")
    print("  0    : No mode/Reset")
    print("SYSTEM:")
    print("  ESC/Ctrl+C : Exit controller")
    print("─────────────────────────────────────")

def show_status(mode, linear_x, linear_y, angular_z):
    print(f"\rMode: {mode.ljust(15)} | Speed: F/B {linear_x:+.2f} L/R {linear_y:+.2f} Rot {angular_z:+.2f}", end='')
    sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    
    # Initialize keyboard handler
    kb = KBHit()
    
    # Control parameters
    linear_x = 0.0
    linear_y = 0.0
    angular_z = 0.0
    current_mode = "None"
    
    # Control increments and limits with proper joystick range
    max_linear_speed = 1.0
    min_linear_speed = -1.0
    max_angular_speed = 1.0
    min_angular_speed = -1.0
    linear_increment = 0.05  # Smaller increment for more control
    angular_increment = 0.1
    
    # Initialize display
    clear_screen()
    print_instructions()
    show_status(current_mode, linear_x, linear_y, angular_z)
    
    # Button state
    buttons = [0] * 11
    
    try:
        last_publish_time = time.time()
        publish_rate = 0.02  # 50Hz publish rate
        
        while rclpy.ok():
            # Spin once to process ROS callbacks
            rclpy.spin_once(node, timeout_sec=0)
            
            # Check for keyboard input
            if kb.kbhit():
                key = kb.getch()
                
                # Process key input
                if key == 'w':
                    linear_x = min(linear_x + linear_increment, max_linear_speed)
                elif key == 's':
                    linear_x = max(linear_x - linear_increment, min_linear_speed)
                elif key == 'a':
                    linear_y = min(linear_y + linear_increment, max_linear_speed)
                elif key == 'd':
                    linear_y = max(linear_y - linear_increment, min_linear_speed)
                elif key == 'q':
                    angular_z = min(angular_z + angular_increment, max_angular_speed)
                elif key == 'e':
                    angular_z = max(angular_z - angular_increment, min_angular_speed)
                elif key == 'x':
                    linear_x = 0.0
                    linear_y = 0.0
                    angular_z = 0.0
                # Mode controls
                elif key == '1':
                    buttons = [0] * 11
                    buttons[0] = 1  # A button
                    current_mode = "In-phase"
                elif key == '2':
                    buttons = [0] * 11
                    buttons[4] = 1  # LB button
                    current_mode = "Opposite-phase"
                elif key == '3':
                    buttons = [0] * 11
                    buttons[5] = 1  # RB button
                    current_mode = "Pivot turn"
                elif key == '0':
                    buttons = [0] * 11
                    current_mode = "None"
                # Exit on ESC or Ctrl+C
                elif key in ('\x03', '\x1b'):
                    break
                
                # Update status display
                show_status(current_mode, linear_x, linear_y, angular_z)
            
            # Publish at regular intervals
            current_time = time.time()
            if current_time - last_publish_time >= publish_rate:
                node.update_joy(linear_x, linear_y, angular_z, buttons)
                # Reset momentary button presses

                last_publish_time = current_time
                
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings
        kb.set_normal_term()
        print("\nKeyboard controller stopped")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()