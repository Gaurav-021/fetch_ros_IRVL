#!/usr/bin/env python
import socket
import struct
from threading import Thread
import rospy
from geometry_msgs.msg import WrenchStamped
import time
import tf
from tf.transformations import quaternion_matrix
import numpy as np

class Sensor:
    '''Class manager for ATI Force/Torque sensor via UDP/RDT with absolute force/torque computation.'''
    def __init__(self, ip="10.42.42.41"):
        '''
        Args:
            ip (str): The IP address of the Net F/T box.
        '''
        # Initialization
        self.ip = ip
        self.port = 49152  # UDP/RDT port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((ip, self.port))
        self.stream = False
        self.cpf = 1000000.0  # Counts per Force (NetFT Configuration)
        self.cpt = 1000000.0  # Counts per Torque (NetFT Configuration)
        
        # Initialize ROS node
        rospy.init_node('ft_sensor', anonymous=True)
        self.pub_raw = rospy.Publisher('/gripper/ft_sensor/raw', WrenchStamped, queue_size=10)
        self.pub_absolute = rospy.Publisher('/gripper/ft_sensor/absolute', WrenchStamped, queue_size=10)
        self.data = None

        # TF listener for pose
        self.tf_listener = tf.TransformListener()
        
        # Gripper properties
        self.gripper_mass = 1.5  # kg
        self.gripper_pos = np.array([0.5, 0.0, 0.0])  # meters in ati_link frame
        self.gravity = 9.81  # m/s^2

        # Reset NetFT software bias and capture pose
        self.send(0x0042)
        self.capture_initial_bias()

        # Subscriber for raw FT data
        self.sub_raw = rospy.Subscriber('/gripper/ft_sensor/raw', WrenchStamped, self.compensate_and_publish)

    def capture_initial_bias(self):
        '''Capture the initial pose of ati_link and compute gripper bias after reset.'''
        try:
            self.tf_listener.waitForTransform("base_link", "ati_link", rospy.Time(0), rospy.Duration(2.0))
            (trans, rot) = self.tf_listener.lookupTransform("base_link", "ati_link", rospy.Time(0))
            self.initial_rot = np.array(rot)  # Quaternion [x, y, z, w]
            rospy.loginfo("Captured initial pose of ati_link relative to base_link")
            
            # Compute gripper bias at reset pose
            rot_matrix = quaternion_matrix(self.initial_rot)[:3, :3]
            gravity_base = np.array([0.0, 0.0, -self.gripper_mass * self.gravity])  # [0, 0, -14.715] N
            self.bias_force = rot_matrix.T.dot(gravity_base)  # Gravity in ati_link frame
            self.bias_torque = np.cross(self.gripper_pos, self.bias_force)  # Torque in ati_link frame
            rospy.loginfo(f"Gripper bias - Force: {self.bias_force}, Torque: {self.bias_torque}")
        except (tf.Exception) as e:
            rospy.logerr(f"Failed to capture initial pose: {e}")
            self.initial_rot = np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion
            self.bias_force = np.array([0.0, 0.0, 0.0])
            self.bias_torque = np.array([0.0, 0.0, 0.0])

    def send(self, command, count=0):
        '''Send a command to the NetFT using UDP/RDT.'''
        header = 0x1234
        message = struct.pack('!HHI', header, command, count)
        self.sock.send(message)

    def receive(self):
        '''Receives and unpacks a response from the Net F/T box.'''
        rawdata = self.sock.recv(1024)
        data = struct.unpack('!IIIiiiiii', rawdata)[3:]
        self.data = [data[i] for i in range(6)]  # Raw counts
        return 

    def receiveHandler(self):
        '''A handler to receive and store data.'''
        while self.stream:
            self.receive()
            self.publish_to_ros()

    def startStreaming(self):
        '''Start Data Stream'''
        self.getMeasurements(0)  # Signals NetFT to stream
        rospy.loginfo("NetFT Stream Started")
        self.stream = True
        self.receiveThread = Thread(target=self.receiveHandler)
        self.receiveThread.daemon = True
        self.receiveThread.start()

    def getMeasurements(self, n):
        '''Request measurements from NetFT'''
        self.send(2, count=n)

    def stopStreaming(self):
        '''Stop NetFT streaming'''
        self.stream = False
        time.sleep(0.1)
        self.send(0)  # Sends signal to NetFT to stop
        
    def publish_to_ros(self):
        '''Publish the current force and torque data to ROS topic /gripper/ft_sensor/raw.'''
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "ati_link"
        
        # URDF to NetFT axis translation
        msg.wrench.force.x = self.data[2] / self.cpf
        msg.wrench.force.y = -1.0 * self.data[0] / self.cpf
        msg.wrench.force.z = -1.0 * self.data[1] / self.cpf
        msg.wrench.torque.x = self.data[5] / self.cpt
        msg.wrench.torque.y = -1.0 * self.data[3] / self.cpt
        msg.wrench.torque.z = -1.0 * self.data[4] / self.cpt
        self.pub_raw.publish(msg)

    def compensate_and_publish(self, raw_msg):
        '''Compute and publish absolute force/torque by removing gripper bias from software reset.'''
        absolute_msg = WrenchStamped()
        absolute_msg.header = raw_msg.header
        absolute_msg.header.frame_id = "ati_link"
        
        # Subtract the gripper bias (part of the software bias) to get absolute external forces/torques
        absolute_msg.wrench.force.x = raw_msg.wrench.force.x - self.bias_force[0]
        absolute_msg.wrench.force.y = raw_msg.wrench.force.y - self.bias_force[1]
        absolute_msg.wrench.force.z = raw_msg.wrench.force.z - self.bias_force[2]
        absolute_msg.wrench.torque.x = raw_msg.wrench.torque.x - self.bias_torque[0]
        absolute_msg.wrench.torque.y = raw_msg.wrench.torque.y - self.bias_torque[1]
        absolute_msg.wrench.torque.z = raw_msg.wrench.torque.z - self.bias_torque[2]
        
        self.pub_absolute.publish(absolute_msg)

if __name__ == "__main__":
    sensor = Sensor()
    try:
        rospy.loginfo("Starting Net FT Streaming..")
        sensor.startStreaming()
        rospy.spin()  # Keep node alive
    finally:
        sensor.stopStreaming()
        rospy.loginfo("Net FT Streaming stopped")