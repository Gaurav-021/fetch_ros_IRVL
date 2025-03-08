#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, WrenchStamped
import keyboard
import threading
import time
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import sys
import tty
import termios
import tf
from tf.transformations import quaternion_matrix

class ArmTeleop:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('arm_teleop_keyboard', anonymous=True)
        
        # Axis mapping
        self.axis_map = {
            'x': rospy.get_param('~axis_x', {'up': 'w', 'down': 's'}),
            'y': rospy.get_param('~axis_y', {'left': 'a', 'right': 'd'}),
            'z': rospy.get_param('~axis_z', {'up': 'q', 'down': 'e'}),
            'roll': rospy.get_param('~axis_roll', {'ccw': 'u', 'cw': 'o'}),
            'pitch': rospy.get_param('~axis_pitch', {'up': 'i', 'down': 'k'}),
            'yaw': rospy.get_param('~axis_yaw', {'left': 'j', 'right': 'l'})
        }

        self.emergency_stop_key = 'backspace'
        
        # Speed levels
        self.speed_levels = [0.01, 0.025, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0]
        self.current_speed_level = 4
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

        # Force-torque limits
        self.max_force = 10.0  # 10 N
        self.max_torque = 2.0  # 1 N·m

        # ROS publisher for arm
        self.cmd_pub = rospy.Publisher('/arm_controller/cartesian_twist/command', 
                                     TwistStamped, 
                                     queue_size=10)

        # TF listener
        self.tf_listener = tf.TransformListener()

        # Gripper setup
        self.gripper_closed = False
        self.last_gripper_toggle_time = time.time()
        self.gripper_debounce_interval = 0.5
        self.min_position = rospy.get_param('~closed_position', 0.0)
        self.max_position = rospy.get_param('~open_position', 0.115)
        self.max_effort = rospy.get_param('~max_effort', 100.0)
        self.gripper_client = actionlib.SimpleActionClient('gripper_controller/gripper_action', 
                                                         GripperCommandAction)
        if not self.gripper_client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr("Gripper action server may not be connected.")

        # Force-torque sensor subscriber
        self.wrench = WrenchStamped()
        self.wrench_lock = threading.Lock()
        self.ft_sub = rospy.Subscriber('/gripper/ft_sensor/external', WrenchStamped, self.ft_callback)

        # State variables
        self.active = True
        self.desired = TwistStamped()
        self.last = TwistStamped()
        self.last_command_time = rospy.Time.now()
        
        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        # Start threads
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.keyboard_thread.start()
        self.publish_thread.start()

    def ft_callback(self, msg):
        """Callback to update force-torque readings."""
        with self.wrench_lock:
            self.wrench = msg

    def keyboard_loop(self):
        while self.running and not rospy.is_shutdown():
            self.update()
            if keyboard.is_pressed(self.emergency_stop_key):
                self.emergency_stop()
                self.running = False
            time.sleep(0.01)

    def update(self):
        self.desired = TwistStamped()
        
        # Speed level check
        current_time = time.time()
        if current_time - self.last_speed_change_time >= self.debounce_interval:
            for i in range(1, 10):
                if keyboard.is_pressed(str(i)):
                    self.current_speed_level = i - 1
                    rospy.loginfo(f"Speed level set to {i} ({self.speed_levels[self.current_speed_level]*100}%)")
                    self.last_speed_change_time = current_time
                    break

        speed_multiplier = self.speed_levels[self.current_speed_level]
        
        # Define twist in gripper_link frame
        self.desired.twist.linear.x = (1.0 if keyboard.is_pressed(self.axis_map['x']['up']) else 
                                     -1.0 if keyboard.is_pressed(self.axis_map['x']['down']) else 0.0) * self.max_vel_x * speed_multiplier
        self.desired.twist.linear.y = (1.0 if keyboard.is_pressed(self.axis_map['y']['right']) else 
                                     -1.0 if keyboard.is_pressed(self.axis_map['y']['left']) else 0.0) * self.max_vel_y * speed_multiplier
        self.desired.twist.linear.z = (1.0 if keyboard.is_pressed(self.axis_map['z']['up']) else 
                                     -1.0 if keyboard.is_pressed(self.axis_map['z']['down']) else 0.0) * self.max_vel_z * speed_multiplier
        self.desired.twist.angular.x = (1.0 if keyboard.is_pressed(self.axis_map['roll']['ccw']) else 
                                      -1.0 if keyboard.is_pressed(self.axis_map['roll']['cw']) else 0.0) * self.max_vel_roll * speed_multiplier
        self.desired.twist.angular.y = (1.0 if keyboard.is_pressed(self.axis_map['pitch']['up']) else 
                                      -1.0 if keyboard.is_pressed(self.axis_map['pitch']['down']) else 0.0) * self.max_vel_pitch * speed_multiplier
        self.desired.twist.angular.z = (1.0 if keyboard.is_pressed(self.axis_map['yaw']['right']) else 
                                      -1.0 if keyboard.is_pressed(self.axis_map['yaw']['left']) else 0.0) * self.max_vel_yaw * speed_multiplier
        
        if any([self.desired.twist.linear.x, self.desired.twist.linear.y, self.desired.twist.linear.z,
                self.desired.twist.angular.x, self.desired.twist.angular.y, self.desired.twist.angular.z]):
            self.last_command_time = rospy.Time.now()

        # Gripper toggle
        if keyboard.is_pressed('space') and (current_time - self.last_gripper_toggle_time) >= self.gripper_debounce_interval:
            self.toggle_gripper()
            self.last_gripper_toggle_time = current_time

    def toggle_gripper(self):
        goal = GripperCommandGoal()
        if self.gripper_closed:
            goal.command.position = self.max_position
            goal.command.max_effort = self.max_effort
            self.gripper_closed = False
            rospy.loginfo("Opening gripper")
        else:
            goal.command.position = self.min_position
            goal.command.max_effort = self.max_effort
            self.gripper_closed = True
            rospy.loginfo("Closing gripper")
        self.gripper_client.send_goal(goal)

    def integrate(self, desired, last, max_acc, dt):
        diff = desired - last
        max_change = max_acc * dt
        return last + max(min(diff, max_change), -max_change)

    def transform_twist(self, twist, from_frame, to_frame):
        try:
            self.tf_listener.waitForTransform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(to_frame, from_frame, rospy.Time(0))
            rot_matrix = quaternion_matrix(rot)[:3, :3]
            linear = [twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z]
            transformed_linear = rot_matrix.dot(linear)
            angular = [twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z]
            transformed_angular = rot_matrix.dot(angular)
            transformed_twist = TwistStamped()
            transformed_twist.header.frame_id = to_frame
            transformed_twist.header.stamp = rospy.Time.now()
            transformed_twist.twist.linear.x = transformed_linear[0]
            transformed_twist.twist.linear.y = transformed_linear[1]
            transformed_twist.twist.linear.z = transformed_linear[2]
            transformed_twist.twist.angular.x = transformed_angular[0]
            transformed_twist.twist.angular.y = transformed_angular[1]
            transformed_twist.twist.angular.z = transformed_angular[2]
            return transformed_twist
        except (tf.Exception) as e:
            rospy.logwarn(f"TF transform failed: {e}")
            return twist

    def limit_twist(self, twist):
        """Limit twist based on force-torque sensor readings."""
        with self.wrench_lock:
            wrench = self.wrench.wrench

        # Check forces and limit linear velocities
        if wrench.force.x > self.max_force and twist.twist.linear.x < 0:
            twist.twist.linear.x = 0.0
            #rospy.logwarn("X force limit exceeded (positive)")
        elif wrench.force.x < -self.max_force and twist.twist.linear.x > 0:
            twist.twist.linear.x = 0.0
            #rospy.logwarn("X force limit exceeded (negative)")
        
        if wrench.force.y > self.max_force and twist.twist.linear.y < 0:
            twist.twist.linear.y = 0.0
            #rospy.logwarn("Y force limit exceeded (positive)")
        elif wrench.force.y < -self.max_force and twist.twist.linear.y > 0:
            twist.twist.linear.y = 0.0
            #rospy.logwarn("Y force limit exceeded (negative)")
        
        if wrench.force.z > self.max_force and twist.twist.linear.z < 0:
            twist.twist.linear.z = 0.0
            #rospy.logwarn("Z force limit exceeded (positive)")
        elif wrench.force.z < -self.max_force and twist.twist.linear.z > 0:
            twist.twist.linear.z = 0.0
            #rospy.logwarn("Z force limit exceeded (negative)")

        # Check torques and limit angular velocities
        if wrench.torque.x > self.max_torque and twist.twist.angular.x < 0:
            twist.twist.angular.x = 0.0
            rospy.logwarn("Roll torque limit exceeded (positive)")
        elif wrench.torque.x < -self.max_torque and twist.twist.angular.x > 0:
            twist.twist.angular.x = 0.0
            #rospy.logwarn("Roll torque limit exceeded (negative)")
        
        if wrench.torque.y > self.max_torque and twist.twist.angular.y < 0:
            twist.twist.angular.y = 0.0
            #rospy.logwarn("Pitch torque limit exceeded (positive)")
        elif wrench.torque.y < -self.max_torque and twist.twist.angular.y > 0:
            twist.twist.angular.y = 0.0
            #rospy.logwarn("Pitch torque limit exceeded (negative)")
        
        if wrench.torque.z > self.max_torque and twist.twist.angular.z < 0:
            twist.twist.angular.z = 0.0
            #rospy.logwarn("Yaw torque limit exceeded (positive)")
        elif wrench.torque.z < -self.max_torque and twist.twist.angular.z > 0:
            twist.twist.angular.z = 0.0
            #rospy.logwarn("Yaw torque limit exceeded (negative)")

        return twist

    def publish_loop(self):
        rate = rospy.Rate(100)
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
                    self.last.header.frame_id = "gripper_link"
                    
                    # Apply force-torque limits
                    limited_twist = self.limit_twist(self.last)
                    
                    # Transform to base_link
                    publish_twist = self.transform_twist(limited_twist, "gripper_link", "base_link")
                    self.cmd_pub.publish(publish_twist)
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
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

if __name__ == '__main__':
    try:
        teleop = ArmTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        teleop.stop()