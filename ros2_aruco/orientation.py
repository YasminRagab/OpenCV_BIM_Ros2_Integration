##This node extracts panel target state from BIM text file.
##It also recieves the transformstamped message for current state pose.
##This node then compare the two states to calculate the required pose alignment.

import rclpy
import math 
import codecs

import numpy as np
import tf_transformations as tf_trans

from rclpy.node         import Node
from tf2_ros            import TransformStamped
from numpy              import mat,block
from tf_transformations import euler_from_quaternion
from ament_index_python import get_package_share_directory

class Orientation(Node):
    def __init__(self,name):
        super().__init__(name)

        # Defining attributes for robot end effector (head manipulator) orientation (current state):
        self.head_roll  = 0.0
        self.head_pitch = 0.0
        self.head_yaw   = 0.0

        # Defining attributes for panel marker orientation (target state):
        self.panel_roll  = 0.0
        self.panel_pitch = 0.0
        self.panel_yaw   = 0.0

        # Defining attributes for required orientation:
        self.required_roll  = 0.0
        self.required_pitch = 0.0
        self.required_yaw   = 0.0

        # creating a subscriber that recieves the head frame transform information in reference to the world (odom) frame:
        self.trans_sub = self.create_subscription(TransformStamped, 'head_transformation', self.trans_callback, 10)

        self.degrees_from_qt()

    def rot_from_txt(self):

        # Retrieving panel orientation (target state) from BIM:

        # reading text from BIM file: 
        path = get_package_share_directory('ros2_aruco') + '/ros2_aruco/panel1.txt'
        bim = codecs.open(path, 'r', 'UTF-8')
        txt = bim.read()

        # removing brackets at the start and end of text: 
        txt_1 = txt[1:]
        txt_2 = txt_1[:-3]

        # splitting lines of text (rotation vectors) and assigning them to attributes: 
        vec_list = txt_2.split(')\r\n(')
        row1 = vec_list[0].split(',')
        row2 = vec_list[1].split(',')
        row3 = vec_list[2].split(',')
        rvec_x = [float(num) for num in row1]
        rvec_y = [float(num) for num in row2]
        rvec_z = [float(num) for num in row3]

        # forming rotation matrix from rotation vectors attributes:
        self.r_matrix = block([rvec_x,
                               rvec_y,
                               rvec_z])

    def qt_from_rmat(self):

        self.rot_from_txt()

        # transforming the rotation matrix from 3x3 to 4x4 as tf_trans.quaternion_from matrix expects a 4x4 matrix:    
        rot_matrix = np.eye(4)
        rot_matrix[0:3, 0:3] = self.r_matrix

        # retrieving quaternion from rotation matrix and assigning to attributes:
        quat = tf_trans.quaternion_from_matrix(rot_matrix)
        self.orient_x = quat[2]
        self.orient_y = quat[3]
        self.orient_z = quat[1]
        self.orient_w = quat[0]

    def degrees_from_qt(self):

        self.qt_from_rmat()

        # converting quaternion to euler in radians for comming adjustment for yaw from revit to match rviz orientation:
        orientation_list = [self.orient_x, self.orient_y , self.orient_z, self.orient_w ]
        (roll_x, pitch_y, yaw_z) = euler_from_quaternion(orientation_list)

        # converting euler from radians to degrees for easier understanding and adjusting yaw to match manipulating head mesh orientation:  
        self.panel_roll = math.degrees(roll_x) 
        self.panel_pitch = 90 - math.degrees(pitch_y) 
        self.panel_yaw = math.degrees(yaw_z) 

        # Adjusting panel orientation from Revit to match rviz orientation direction:
        if self.panel_roll > 0:
            self.panel_roll = self.panel_roll - 180
        elif self.panel_roll < 0:
            self.panel_roll = self.panel_roll + 180

        if self.panel_yaw > 0:
            self.panel_yaw = self.panel_yaw - 180
        elif self.panel_yaw < 0:
            self.panel_yaw = self.panel_yaw + 180

    def trans_callback(self, msg):

        # Retrieving head rotation (current state) from TransformStamped message and converting it from quaternion to euler radians:
        head_trans = msg
        trans_quat = [head_trans.transform.rotation.x, head_trans.transform.rotation.y, head_trans.transform.rotation.z , head_trans.transform.rotation.w]
        (t_roll_x, t_pitch_y, t_yaw_z) = euler_from_quaternion(trans_quat)

        # Converting euler radians to degrees for easier understanding:
        self.head_roll  = math.degrees(t_roll_x)
        self.head_pitch = math.degrees(t_pitch_y)
        self.head_yaw   = math.degrees(t_yaw_z)

        # Calculating required orientation from current state (head) to target state (BIM):
        self.required_roll  = self.panel_roll - self.head_roll 
        self.required_pitch = self.panel_pitch - self.head_pitch
        self.required_yaw   = self.panel_yaw - self.head_yaw

        # Real-time orientation values display:
        self.get_logger().info(f"head_roll: {self.head_roll},head_pitch: {self.head_pitch},head_yaw: {self.head_yaw}")  
        self.get_logger().info(f"panel_roll: {self.panel_roll},panel_pitch: {self.panel_pitch},panel_yaw: {self.panel_yaw}")
        self.get_logger().info(f"required_roll: {self.required_roll},required_pitch: {self.required_pitch},required_yaw: {self.required_yaw}")  

def main(args=None):
    rclpy.init(args=args)
    orientation = Orientation('orientation')
    try:
        rclpy.spin(orientation)
    except KeyboardInterrupt:
        print(f"quitting the program...")
        orientation.destroy_node()
    exit(0)

if __name__=='__main__':
    main()
