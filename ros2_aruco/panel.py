import rclpy
import math
import codecs

import numpy as np
import tf_transformations as tf_trans

from numpy                   import mat,block
from rclpy.node              import Node
from visualization_msgs.msg  import Marker, MarkerArray
from ament_index_python      import get_package_share_directory
from tf_transformations      import euler_from_quaternion
from tf2_geometry_msgs       import PoseStamped
from tf2_ros                 import TransformStamped

class GlassPanel(Node):
    def __init__(self,name):
        super().__init__(name)

        self.pose_stamped = PoseStamped()

        # Defining attributes for robot end effector (head manipulator) orientation (current state):
        self.head_roll  = 0.0
        self.head_pitch = 0.0
        self.head_yaw   = 0.0

        # creating a publisher for panel marker for rviz visualization:
        self.pub_rviz  = self.create_publisher(MarkerArray, 'markers', 10)
        self.trans_sub = self.create_subscription(TransformStamped, 'head_transformation', self.trans_callback, 10)
        self.timer     = self.create_timer(2.0, self.timer_callback)

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

        self.text.pose.position.x = 0.75
        self.text.pose.position.y = 1.50
        self.text.pose.position.z = 2.90

        self.text.pose.orientation.x = 0.0
        self.text.pose.orientation.y = 0.0
        self.text.pose.orientation.z = 0.0
        self.text.pose.orientation.w = 1.0

        self.text.scale.x = 1.0
        self.text.scale.y = 1.0
        self.text.scale.z = 0.2

        self.text.color.a = 1.0
        self.text.color.r = 1.0
        self.text.color.g = 1.0
        self.text.color.b = 1.0

        self.text.text = f"Target"
        self.array.markers.append(self.text)

        # Create a marker message to display the text
        self.marker = Marker()
        self.marker.header.frame_id = 'world'
        self.marker.ns = ''
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.id = 3
        self.marker.type = Marker().TEXT_VIEW_FACING
        
        self.marker.pose = self.pose_stamped.pose

        self.text.scale.x   = 1.0
        self.text.scale.y   = 1.0
        self.marker.scale.z = 0.2

        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0

        self.marker.text = f"{self.head_roll}"
        self.array.markers.append(self.marker)

    def trans_callback(self, msg):

        # Retrieving head rotation (current state) from TransformStamped message and converting it from quaternion to euler radians:
        head_trans = msg
        trans_quat = [head_trans.transform.rotation.x, head_trans.transform.rotation.y, head_trans.transform.rotation.z , head_trans.transform.rotation.w]
        (t_roll_x, t_pitch_y, t_yaw_z) = euler_from_quaternion(trans_quat)

        # Converting euler radians to degrees for easier understanding:
        self.head_roll  = math.degrees(t_roll_x)
        self.head_pitch = math.degrees(t_pitch_y)
        self.head_yaw   = math.degrees(t_yaw_z)

        # self.get_logger().info(f"head_roll: {self.head_roll},head_pitch: {self.head_pitch},head_yaw: {self.head_yaw}")  

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