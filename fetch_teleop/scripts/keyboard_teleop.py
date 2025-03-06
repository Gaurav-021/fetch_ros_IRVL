#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
import keyboard
import threading
import time
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import sys
import tty
import termios

class ArmTeleop:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('arm_teleop_keyboard', anonymous=True)
        
        # Existing axis mapping
        self.axis_map = {
            'x': rospy.get_param('~axis_x', {'up': 'w', 'down': 's'}),
            'y': rospy.get_param('~axis_y', {'left': 'a', 'right': 'd'}),
            'z': rospy.get_param('~axis_z', {'up': 'q', 'down': 'e'}),
            'roll': rospy.get_param('~axis_roll', {'ccw': 'u', 'cw': 'o'}),
            'pitch': rospy.get_param('~axis_pitch', {'up': 'i', 'down': 'k'}),
            'yaw': rospy.get_param('~axis_yaw', {'left': 'j', 'right': 'l'})
        }

        self.emergency_stop_key = 'backspace'
        
        # Speed levels (expanded to 9)
        self.speed_levels = [0.01, 0.025, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0]  # 20% to 100%
        self.current_speed_level = 4  # Default to middle speed (60%)
        self.last_speed_change_time = time.time()
        self.debounce_interval = 0.2

        # Velocity and acceleration limits
        self.max_vel_x = rospy.get_param('~max_vel_x', 1.0)
        self.max_vel_y = rospy.get_param('~max_vel_y', 1.0)
        self.max_vel_z = rospy.get_param('~max_vel_z', 1.0)
        self.max_acc_x = rospy.get_param('~max_acc_x', 10.0)
        self.max_acc_y = rospy.get_param('~max_acc_y', 10.0)
        self.max_acc_z = rospy.get_param('~max_acc_z', 10.0)
        self.max_vel_roll = rospy.get_param('~max_vel_roll', 2.0)
        self.max_vel_pitch = rospy.get_param('~max_vel_pitch', 2.0)
        self.max_vel_yaw = rospy.get_param('~max_vel_yaw', 2.0)
        self.max_acc_roll = rospy.get_param('~max_acc_roll', 10.0)
        self.max_acc_pitch = rospy.get_param('~max_acc_pitch', 10.0)
        self.max_acc_yaw = rospy.get_param('~max_acc_yaw', 10.0)

        # ROS publisher for arm
        self.cmd_pub = rospy.Publisher('/arm_controller/cartesian_twist/command', 
                                     TwistStamped, 
                                     queue_size=10)

        # Gripper setup
        self.gripper_closed = False  # Initial state: open
        self.last_gripper_toggle_time = time.time()
        self.gripper_debounce_interval = 0.5  # 500ms debounce for gripper toggle
        self.min_position = rospy.get_param('~closed_position', 0.0)  # Closed position
        self.max_position = rospy.get_param('~open_position', 0.115)  # Open position
        self.max_effort = rospy.get_param('~max_effort', 100.0)       # Max effort
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/gripper_action', 
                                                         GripperCommandAction)
        if not self.gripper_client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr("Gripper action server may not be connected.")

        # State variables
        self.active = True
        self.desired = TwistStamped()
        self.last = TwistStamped()
        self.last_command_time = rospy.Time.now()
        
        # Store terminal settings to disable echo
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())  # Disable line buffering and echo
        
        # Start threads
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.keyboard_thread.start()
        self.publish_thread.start()

    def keyboard_loop(self):
        while self.running and not rospy.is_shutdown():
            self.update()
            if keyboard.is_pressed(self.emergency_stop_key):
                self.emergency_stop()
                self.running = False
            time.sleep(0.01)  # 100 Hz polling

    def update(self):
        self.desired = TwistStamped()
        
        # Speed level check (now 1-9)
        current_time = time.time()
        if current_time - self.last_speed_change_time >= self.debounce_interval:
            for i in range(1, 10):  # Changed to 1-9
                if keyboard.is_pressed(str(i)):
                    self.current_speed_level = i - 1
                    rospy.loginfo(f"Speed level set to {i} ({self.speed_levels[self.current_speed_level]*100}%)")
                    self.last_speed_change_time = current_time
                    break

        speed_multiplier = self.speed_levels[self.current_speed_level]
        
        # Linear control
        self.desired.twist.linear.x = (1.0 if keyboard.is_pressed(self.axis_map['x']['up']) else 
                                     -1.0 if keyboard.is_pressed(self.axis_map['x']['down']) else 0.0) * self.max_vel_x * speed_multiplier
        self.desired.twist.linear.y = (1.0 if keyboard.is_pressed(self.axis_map['y']['right']) else 
                                     -1.0 if keyboard.is_pressed(self.axis_map['y']['left']) else 0.0) * self.max_vel_y * speed_multiplier
        self.desired.twist.linear.z = (1.0 if keyboard.is_pressed(self.axis_map['z']['up']) else 
                                     -1.0 if keyboard.is_pressed(self.axis_map['z']['down']) else 0.0) * self.max_vel_z * speed_multiplier
        
        # Angular control
        self.desired.twist.angular.x = (1.0 if keyboard.is_pressed(self.axis_map['roll']['ccw']) else 
                                      -1.0 if keyboard.is_pressed(self.axis_map['roll']['cw']) else 0.0) * self.max_vel_roll * speed_multiplier
        self.desired.twist.angular.y = (1.0 if keyboard.is_pressed(self.axis_map['pitch']['up']) else 
                                      -1.0 if keyboard.is_pressed(self.axis_map['pitch']['down']) else 0.0) * self.max_vel_pitch * speed_multiplier
        self.desired.twist.angular.z = (1.0 if keyboard.is_pressed(self.axis_map['yaw']['right']) else 
                                      -1.0 if keyboard.is_pressed(self.axis_map['yaw']['left']) else 0.0) * self.max_vel_yaw * speed_multiplier
        
        if any([self.desired.twist.linear.x, self.desired.twist.linear.y, self.desired.twist.linear.z,
                self.desired.twist.angular.x, self.desired.twist.angular.y, self.desired.twist.angular.z]):
            self.last_command_time = rospy.Time.now()

        # Gripper toggle check (event-based)
        if keyboard.is_pressed('space') and (current_time - self.last_gripper_toggle_time) >= self.gripper_debounce_interval:
            self.toggle_gripper()
            self.last_gripper_toggle_time = current_time

    def toggle_gripper(self):
        goal = GripperCommandGoal()
        if self.gripper_closed:
            # Open the gripper
            goal.command.position = self.max_position
            goal.command.max_effort = self.max_effort
            self.gripper_closed = False
            rospy.loginfo("Opening gripper")
        else:
            # Close the gripper
            goal.command.position = self.min_position
            goal.command.max_effort = self.max_effort
            self.gripper_closed = True
            rospy.loginfo("Closing gripper")
        self.gripper_client.send_goal(goal)

    def integrate(self, desired, last, max_acc, dt):
        diff = desired - last
        max_change = max_acc * dt
        return last + max(min(diff, max_change), -max_change)

    def publish_loop(self):
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown() and self.running:
            if self.active:
                if (rospy.Time.now() - self.last_command_time).to_sec() > 0.5:
                    self.desired = TwistStamped()
                
                dt = (rospy.Time.now() - self.last.header.stamp).to_sec()
                if dt > 0:
                    self.last.twist.linear.x = self.integrate(self.desired.twist.linear.x, 
                                                            self.last.twist.linear.x, 
                                                            self.max_acc_x, dt)
                    self.last.twist.linear.y = self.integrate(self.desired.twist.linear.y, 
                                                            self.last.twist.linear.y, 
                                                            self.max_acc_y, dt)
                    self.last.twist.linear.z = self.integrate(self.desired.twist.linear.z, 
                                                            self.last.twist.linear.z, 
                                                            self.max_acc_z, dt)
                    self.last.twist.angular.x = self.integrate(self.desired.twist.angular.x, 
                                                             self.last.twist.angular.x, 
                                                             self.max_acc_roll, dt)
                    self.last.twist.angular.y = self.integrate(self.desired.twist.angular.y, 
                                                             self.last.twist.angular.y, 
                                                             self.max_acc_pitch, dt)
                    self.last.twist.angular.z = self.integrate(self.desired.twist.angular.z, 
                                                             self.last.twist.angular.z, 
                                                             self.max_acc_yaw, dt)
                    
                    self.last.header.stamp = rospy.Time.now()
                    self.last.header.frame_id = "base_link"
                    self.cmd_pub.publish(self.last)
            rate.sleep()

    def emergency_stop(self):
        self.desired = TwistStamped()
        self.last = TwistStamped()
        for _ in range(5):
            self.cmd_pub.publish(self.last)
            time.sleep(0.05)
        self.active = False

    def stop(self):
        self.emergency_stop()
        self.running = False
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

if __name__ == '__main__':
    try:
        teleop = ArmTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        teleop.stop()