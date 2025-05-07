#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped, WrenchStamped
import threading
import tf
from tf.transformations import quaternion_matrix

class ArmTeleop:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('arm_force_feedback_control', anonymous=True)

        # Control gains and thresholds
        self.k_f = 0.01  # Linear velocity gain: m/(N s)
        self.k_t = 0.1   # Angular velocity gain: rad/(Nm s)
        self.force_threshold = 0.5  # N, deadband for forces
        self.torque_threshold = 0.05  # Nm, deadband for torques

        # Velocity and acceleration limits
        self.max_vel_x = rospy.get_param('~max_vel_x', 1.0)
        self.max_vel_y = rospy.get_param('~max_vel_y', 1.0)
        self.max_vel_z = rospy.get_param('~max_vel_z', 1.0)
        self.max_vel_roll = rospy.get_param('~max_vel_roll', 2.0) / 1.5
        self.max_vel_pitch = rospy.get_param('~max_vel_pitch', 2.0) / 1.5
        self.max_vel_yaw = rospy.get_param('~max_vel_yaw', 2.0) / 1.5
        self.max_acc_x = rospy.get_param('~max_acc_x', 10.0)
        self.max_acc_y = rospy.get_param('~max_acc_y', 10.0)
        self.max_acc_z = rospy.get_param('~max_acc_z', 10.0)
        self.max_acc_roll = rospy.get_param('~max_acc_roll', 10.0)
        self.max_acc_pitch = rospy.get_param('~max_acc_pitch', 10.0)
        self.max_acc_yaw = rospy.get_param('~max_acc_yaw', 10.0)

        # ROS publisher for arm
        self.cmd_pub = rospy.Publisher('/arm_controller/cartesian_twist/command', TwistStamped, queue_size=10)

        # TF listener
        self.tf_listener = tf.TransformListener()

        # Force-torque sensor subscriber
        self.wrench = WrenchStamped()
        self.ft_sub = rospy.Subscriber('/gripper/ft_sensor/external', WrenchStamped, self.ft_callback)

        # State variables
        self.last = TwistStamped()
        self.last.header.stamp = rospy.Time.now()
        self.last.header.frame_id = "base_link"

        # Start publish thread
        self.running = True
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.start()

    def ft_callback(self, msg):
        """Callback to update force-torque readings."""
        self.wrench = msg

    def transform_twist(self, twist, from_frame, to_frame):
        """Transform both linear and angular velocities from one frame to another."""
        try:
            self.tf_listener.waitForTransform(to_frame, from_frame, twist.header.stamp, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(to_frame, from_frame, twist.header.stamp)
            rot_matrix = quaternion_matrix(rot)[:3, :3]
            linear = [twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z]
            transformed_linear = rot_matrix.dot(linear)
            angular = [twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z]
            transformed_angular = rot_matrix.dot(angular)
            transformed_twist = TwistStamped()
            transformed_twist.header.frame_id = to_frame
            transformed_twist.header.stamp = twist.header.stamp
            transformed_twist.twist.linear.x = transformed_linear[0]
            transformed_twist.twist.linear.y = transformed_linear[1]
            transformed_twist.twist.linear.z = transformed_linear[2]
            transformed_twist.twist.angular.x = transformed_angular[0]
            transformed_twist.twist.angular.y = transformed_angular[1]
            transformed_twist.twist.angular.z = transformed_angular[2]
            return transformed_twist
        except (tf.Exception) as e:
            rospy.logwarn("TF transform failed: %s" % e)
            return twist

    def integrate(self, desired, last, max_acc, dt):
        """Apply acceleration limits to velocity changes."""
        diff = desired - last
        max_change = max_acc * dt
        return last + max(min(diff, max_change), -max_change)

    def publish_loop(self):
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown() and self.running:
            # Compute desired twist in gripper_link frame based on force-torque readings
            desired = TwistStamped()
            desired.header.frame_id = "gripper_link"
            desired.header.stamp = rospy.Time.now()
            wrench = self.wrench.wrench

            # Linear velocities with deadband
            desired.twist.linear.x = (-self.k_f * wrench.force.x if abs(wrench.force.x) > self.force_threshold else 0.0)
            desired.twist.linear.y = (-self.k_f * wrench.force.y if abs(wrench.force.y) > self.force_threshold else 0.0)
            desired.twist.linear.z = (-self.k_f * wrench.force.z if abs(wrench.force.z) > self.force_threshold else 0.0)

            # Angular velocities with deadband
            desired.twist.angular.x = (-self.k_t * wrench.torque.x if abs(wrench.torque.x) > self.torque_threshold else 0.0)
            desired.twist.angular.y = (-self.k_t * wrench.torque.y if abs(wrench.torque.y) > self.torque_threshold else 0.0)
            desired.twist.angular.z = (-self.k_t * wrench.torque.z if abs(wrench.torque.z) > self.torque_threshold else 0.0)

            # Clamp velocities to maximum limits
            desired.twist.linear.x = max(min(desired.twist.linear.x, self.max_vel_x), -self.max_vel_x)
            desired.twist.linear.y = max(min(desired.twist.linear.y, self.max_vel_y), -self.max_vel_y)
            desired.twist.linear.z = max(min(desired.twist.linear.z, self.max_vel_z), -self.max_vel_z)
            desired.twist.angular.x = max(min(desired.twist.angular.x, self.max_vel_roll), -self.max_vel_roll)
            desired.twist.angular.y = max(min(desired.twist.angular.y, self.max_vel_pitch), -self.max_vel_pitch)
            desired.twist.angular.z = max(min(desired.twist.angular.z, self.max_vel_yaw), -self.max_vel_yaw)

            # Transform twist to base_link frame
            transformed_desired = self.transform_twist(desired, "gripper_link", "base_link")

            # Apply acceleration limits
            dt = (rospy.Time.now() - self.last.header.stamp).to_sec()
            if dt > 0:
                self.last.twist.linear.x = self.integrate(transformed_desired.twist.linear.x, self.last.twist.linear.x, self.max_acc_x, dt)
                self.last.twist.linear.y = self.integrate(transformed_desired.twist.linear.y, self.last.twist.linear.y, self.max_acc_y, dt)
                self.last.twist.linear.z = self.integrate(transformed_desired.twist.linear.z, self.last.twist.linear.z, self.max_acc_z, dt)
                self.last.twist.angular.x = self.integrate(transformed_desired.twist.angular.x, self.last.twist.angular.x, self.max_acc_roll, dt)
                self.last.twist.angular.y = self.integrate(transformed_desired.twist.angular.y, self.last.twist.angular.y, self.max_acc_pitch, dt)
                self.last.twist.angular.z = self.integrate(transformed_desired.twist.angular.z, self.last.twist.angular.z, self.max_acc_yaw, dt)

            self.last.header.stamp = rospy.Time.now()
            self.last.header.frame_id = "base_link"

            # Publish the command
            self.cmd_pub.publish(self.last)
            rate.sleep()

    def stop(self):
        """Stop the control loop and thread."""
        self.running = False
        self.publish_thread.join()

if __name__ == '__main__':
    try:
        teleop = ArmTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        teleop.stop()