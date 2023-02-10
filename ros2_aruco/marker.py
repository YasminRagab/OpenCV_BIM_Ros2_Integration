from visualization_msgs.msg  import Marker
import rclpy
from rclpy.node import Node
import codecs
import numpy as np
from numpy import mat,block
import tf_transformations as tf_trans
import os
from ament_index_python import get_package_share_directory
from scipy.spatial.transform import Rotation

class GlassPanel(Node):
    def __init__(self,name):
        super().__init__(name)

        self.panel_pub = self.create_publisher(Marker, 'view', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.panel = Marker()
        self.panel.header.frame_id = 'world'
        self.panel.ns = ''
        self.panel.header.stamp = self.get_clock().now().to_msg()
        self.panel.id = 0
        self.panel.type = Marker().MESH_RESOURCE
        self.panel.action = Marker().ADD

        self.panel.pose.position.x = -4.0
        self.panel.pose.position.y = 0.0
        self.panel.pose.position.z = 0.0

        self.panel.pose.orientation.x = 0.0
        self.panel.pose.orientation.y = 0.0
        self.panel.pose.orientation.z = 0.0
        self.panel.pose.orientation.w = 1.0

        self.panel.scale.x = 1.0
        self.panel.scale.y = 1.0
        self.panel.scale.z = 1.0

        self.panel.color.a = 1.0
        self.panel.color.r = 1.0
        self.panel.color.g = 1.0
        self.panel.color.b = 1.0

        self.panel.mesh_resource = "package://ros2_aruco/urdf_model/Meshes/3d_mesh.dae"
        
    def timer_callback(self):
        self.panel_pub.publish(self.panel)

def main(args=None):
    rclpy.init(args=args)
    marker = GlassPanel('marker')
    try:
        rclpy.spin(marker)
    except KeyboardInterrupt:
        print(f"quitting the program...")
        panel.destroy_node()
    exit(0)


if __name__=='__main__':
    main()
