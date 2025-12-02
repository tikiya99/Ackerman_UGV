#!/usr/bin/env python3
"""
UGV Teleop Keyboard Node

Provides keyboard-based teleoperation for the UGV.
Publishes cmd_vel messages based on keyboard input.

Key bindings:
    i/k - Increase/decrease linear velocity
    j/l - Turn left/right
    u/o/m/. - Diagonal movements
    space - Emergency stop
    q/z - Increase/decrease max velocities
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Key bindings
MSG = """
========================================
UGV Teleop Keyboard Controller
========================================

Movement Controls:
   u    i    o
   j    k    l
   m    ,    .

u/o : forward left/right
i : forward
j/l : turn left/right
k : stop
m/. : backward left/right
, : backward

space : emergency stop (zero all velocities)
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

Current Settings:
"""

# Movement keys
moveBindings = {
    'i': (1, 0),   # Forward
    'o': (1, -1),  # Forward right
    'j': (0, 1),   # Turn left
    'l': (0, -1),  # Turn right
    'u': (1, 1),   # Forward left
    ',': (-1, 0),  # Backward
    '.': (-1, -1), # Backward right
    'm': (-1, 1),  # Backward left
}

# Speed adjustment keys
speedBindings = {
    'q': (1.1, 1.1),   # Increase both
    'z': (0.9, 0.9),   # Decrease both
    'w': (1.1, 1.0),   # Increase linear only
    'x': (0.9, 1.0),   # Decrease linear only
    'e': (1.0, 1.1),   # Increase angular only
    'c': (1.0, 0.9),   # Decrease angular only
}


def getKey(settings):
    """Get a single keypress from keyboard"""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    """Print current velocity settings"""
    return f"Linear: {speed:.2f} m/s | Angular: {turn:.2f} rad/s"


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('ugv_teleop_keyboard')
        
        # Publisher
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 1.0)
        
        self.speed = self.get_parameter('speed').value
        self.turn = self.get_parameter('turn').value
        
        # Current velocities
        self.x = 0.0
        self.th = 0.0
        
        # Control state
        self.status = 0
        
        self.get_logger().info('UGV Teleop Keyboard Node Started')
        self.get_logger().info(vels(self.speed, self.turn))

    def publish_twist(self, x, th):
        """Publish Twist message"""
        twist = Twist()
        twist.linear.x = x * self.speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th * self.turn
        self.pub.publish(twist)

    def run(self):
        """Main loop for keyboard input"""
        settings = termios.tcgetattr(sys.stdin)
        
        try:
            print(MSG)
            print(vels(self.speed, self.turn))
            
            while True:
                key = getKey(settings)
                
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                    self.status += 1
                    
                elif key in speedBindings.keys():
                    self.speed *= speedBindings[key][0]
                    self.turn *= speedBindings[key][1]
                    self.status += 1
                    
                    # Clip speeds to reasonable limits
                    self.speed = max(0.1, min(2.0, self.speed))
                    self.turn = max(0.1, min(3.0, self.turn))
                    
                    if self.status == 15:
                        print(vels(self.speed, self.turn))
                        self.status = 0
                        
                elif key == 'k' or key == ' ':
                    # Stop
                    self.x = 0.0
                    self.th = 0.0
                    if key == ' ':
                        print("EMERGENCY STOP")
                        
                elif key == '\x03':  # Ctrl-C
                    break
                    
                else:
                    # Unknown key
                    if key == '\x03':
                        break
                    continue
                
                # Publish
                self.publish_twist(self.x, self.th)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            
        finally:
            # Stop on exit
            self.publish_twist(0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    
    teleop = TeleopKeyboard()
    
    # Run teleop in main thread
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
