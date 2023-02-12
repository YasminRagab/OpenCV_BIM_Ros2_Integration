#!/usr/bin/env python3
from visualization_msgs.msg  import Marker, MarkerArray
import rclpy
from rclpy.node import Node
import codecs
import numpy as np
from numpy import mat,block
import tf_transformations as tf_trans
from ament_index_python import get_package_share_directory
from tf_transformations import euler_from_quaternion
import math



class GlassPanel(Node):
    def __init__(self,name):
        super().__init__(name)

        self.degrees_from_qt()

        # creating a publisher for panel marker for rviz visualization:

        self.pub_rviz = self.create_publisher(MarkerArray, 'markers', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

        # we are using a marker array visualization message:

        self.array  = MarkerArray()

        # setting up the marker message attributes:
        self.panel  = Marker()
        self.panel.header.frame_id = 'world'
        self.panel.ns = ''
        self.panel.header.stamp = self.get_clock().now().to_msg()
        self.panel.id = 0
        self.panel.type = Marker().MESH_RESOURCE
        self.panel.action = Marker().ADD

        self.panel.pose.position.x = -2.0
        self.panel.pose.position.y = 0.0
        self.panel.pose.position.z = 0.0

        self.panel.pose.orientation.x = 0.0
        self.panel.pose.orientation.y = 0.0
        self.panel.pose.orientation.z = 0.0
        self.panel.pose.orientation.w = 1.0

        self.panel.scale.x = 1.0
        self.panel.scale.y = 1.0
        self.panel.scale.z = 1.0

        self.panel.color.a = 0.5
        self.panel.color.r = 0.0
        self.panel.color.g = 0.0
        self.panel.color.b = 1.0

        # setting the path for the panel marker mesh: 

        self.panel.mesh_resource = "package://ros2_aruco/urdf_model/Meshes/panel.dae"
        self.array.markers.append(self.panel)

        ### 3dview marker
        self.view = Marker()
        self.view.header.frame_id = 'world'
        self.view.ns = ''
        self.view.header.stamp = self.get_clock().now().to_msg()
        self.view.id = 1
        self.view.type = Marker().MESH_RESOURCE
        self.view.action = Marker().ADD

        self.view.pose.position.x = -2.0
        self.view.pose.position.y = 0.0
        self.view.pose.position.z = 0.0

        self.view.pose.orientation.x = 0.0
        self.view.pose.orientation.y = 0.0
        self.view.pose.orientation.z = 0.0
        self.view.pose.orientation.w = 1.0

        self.view.scale.x = 1.0
        self.view.scale.y = 1.0
        self.view.scale.z = 1.0

        self.view.color.a = 1.0
        self.view.color.r = 1.0
        self.view.color.g = 1.0
        self.view.color.b = 1.0

        self.view.mesh_resource = "package://ros2_aruco/urdf_model/Meshes/3d_mesh.dae"
        self.array.markers.append(self.view)

        ### Text marker

        self.text = Marker()
        self.text.header.frame_id = 'world'
        self.text.ns = ''
        self.text.header.stamp = self.get_clock().now().to_msg()
        self.text.id = 2          
        self.text.type = Marker().TEXT_VIEW_FACING

        self.text.pose.position.x = 0.0
        self.text.pose.position.y = 0.0
        self.text.pose.position.z = 2.5 

        self.text.pose.orientation.x = 0.0
        self.text.pose.orientation.y = 0.0
        self.text.pose.orientation.z = 0.0
        self.text.pose.orientation.w = 1.0

        self.text.scale.x = 0.01
        self.text.scale.y = 0.01
        self.text.scale.z = 0.1

        self.text.color.a = 1.0
        self.text.color.r = 1.0
        self.text.color.g = 1.0
        self.text.color.b = 1.0

        self.text.text = f"The target oreintation is\nrotation around x-axis by\n{round(self.euler_x,4)} degrees"

        self.array.markers.append(self.text)

    def rot_from_txt(self):

        # reading text from BIM file: 

        path = get_package_share_directory('ros2_aruco') + '/ros2_aruco/panel1.txt'
        bim  = codecs.open(path, 'r', 'UTF-8')
        txt  = bim.read()

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
        self.orient_x = quat[0]
        self.orient_y = quat[1]
        self.orient_z = quat[2]
        self.orient_w = quat[3]

    def degrees_from_qt(self):

        self.qt_from_rmat()

        # converting quaternion to euler in radians for comming adjustment for yaw from revit to match rviz orientation:

        orientation_list = [self.orient_x, self.orient_y , self.orient_z, self.orient_w ]
        (roll_x, pitch_y, yaw_z) = euler_from_quaternion(orientation_list)

        # converting euler from radians to degrees for easier understanding and adjusting yaw from revit to match rviz orientation:  

        self.euler_x = math.degrees(roll_x) 
        self.euler_y = math.degrees(pitch_y) 
        self.euler_z = math.degrees(yaw_z) 

    def timer_callback(self):

        self.pub_rviz.publish(self.array)    

def main(args=None):
    rclpy.init(args=args)
    panel = GlassPanel('panel')
    try:
        rclpy.spin(panel)
    except KeyboardInterrupt:
        print(f"quitting the program...")
        panel.destroy_node()
    exit(0)


if __name__=='__main__':
    main()