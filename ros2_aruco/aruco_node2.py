##This node estimates Aruco marker pose from usb-cam and publishes a pose message

import rclpy
import cv2
import argparse
import sys
import math 

import numpy as np
import tf_transformations as tf_trans

from rclpy.node                  import Node
from cv_bridge                   import CvBridge, CvBridgeError
from std_msgs.msg                import String, Float32
from sensor_msgs.msg             import Image, CameraInfo
from geometry_msgs.msg           import PoseArray, Pose, Quaternion
from ros2_aruco_interfaces.msg   import ArucoMarkers
from .markers.tag_definitions    import ARUCO_DICT 
from tf_transformations          import euler_from_quaternion

class ArucoNode(Node):

    def __init__(self, args):
        super().__init__('aruco_node')

        # Set up parameters
        self.declare_parameter("marker_size", .0515)  
        self.declare_parameter("camera_frame", "default_cam")
        self.marker_size   = self.get_parameter("marker_size").get_parameter_value().double_value
        self.camera_frame  = self.get_parameter("camera_frame").get_parameter_value().string_value

        # Set up subscriptions
        self.info_sub  = self.create_subscription(CameraInfo, "camera_info", self.info_callback, 10)
        self.image_sub = self.create_subscription(Image, "image_raw", self.image_callback, 10)

        # Set up publishers
        self.poses_pub   = self.create_publisher(Pose, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)
        self.publisher   = self.create_publisher(String,'num_markers',10)

        # Set up fields for camera parameters
        self.info_msg      = None
        self.intrinsic_mat = None
        self.distortion    = None

        # Set up Aruco dictionary
        self.aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[args['type']])
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    def info_callback(self, info_msg):

        #Set up camera coefficients
        self.info_msg      = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion    = np.array(self.info_msg.d)

    def image_callback(self, img_msg):

        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except CvBridgeError as e:
            print(e)

        # lists of ids and the corners beloning to each id
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)
        
        # If there are markers found by detector
        if np.all(ids is not None):
            msg      = String()
            msg.data = str(len(ids))
            self.publisher.publish(msg)

        markers    = ArucoMarkers()
        pose_array = PoseArray()

        if self.camera_frame is None:
            markers.header.frame_id    = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id    = self.camera_frame
            pose_array.header.frame_id = self.camera_frame
            
        markers.header.stamp    = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        # lists of ids and the corners beloning to each id
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        # Pose estimation for marker ID[1]
        marker_id1 = np.array([1])
        if marker_ids is not None:
            if marker_id1 in marker_ids:

                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                if cv2.__version__ > '4.0.0':
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)
                else:
                    rvecs, tvecs    = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.intrinsic_mat, self.distortion)

                #Get marker pose translation vector
                pose = Pose()
                pose.position.x = tvecs[0][0][0]
                pose.position.y = tvecs[0][0][1]
                pose.position.z = tvecs[0][0][2]
                
                #Get marker pose rotation vector
                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[0][0]))[0]

                #Get quaternion from rotation matrix
                quat = tf_trans.quaternion_from_matrix(rot_matrix)

                #Marker orientation in quaternion
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                #Publish aruco marker pose
                self.poses_pub.publish(pose)

def main():

    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('-t', '--type', type=str,
                    default='DICT_ARUCO_ORIGINAL',
                    help='type of ArUCo tag to generate')

    args, unknown = ap.parse_known_args()
    args = vars(args)

    # verify that the supplied ArUCo tag exists and is supported by OpenCV
    if ARUCO_DICT.get(args['type'], None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(args['type']))
        print('Supported Types Include:')
        print(str(ARUCO_DICT.keys()))
        sys.exit(0)

    rclpy.init(args=None)
    node = ArucoNode(args)
    rclpy.spin(node)


if __name__ == '__main__':
    main()