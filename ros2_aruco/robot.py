#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Quaternion, Vector3, PoseArray, Pose
from tf2_ros import TransformBroadcaster, TransformStamped
import tf_transformations as tf_trans
from tf_transformations import euler_from_quaternion, quaternion_multiply
import numpy as np
import math # Math library
from visualization_msgs.msg  import Marker
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException



class MyRobot(Node):
    def __init__(self,name):
        super().__init__(name)

        # Creating a timer for a log info callback:

        # self.create_timer(0.5, self.timer_callback)

        # Creating a speed factor attribute for slowing the speed recieved from the joystick to control the robot movement:

        self.speed_factor = 0.06

        # Defining attributes for robot base horizontal movement:

        self.x = 0.0
        self.y = 0.0

        # Defining attribute for robot arm horizontal rotation, vertical tiliting and extension:

        self.arm_rot = 0.0
        self.arm_tilt = 0.0
        self.arm_ext = 0.0

        #

        self.cv_roll = 0.0
        self.cv_pitch = 0.0
        self.cv_yaw = 0.0

        #

        self.trans_roll = 0.0
        self.trans_pitch = 0.0
        self.trans_yaw = 0.0



        # Defining attributes for robot end effector (head manipulator) orientation:

        self.head_roll = 0.0
        self.head_pitch = 0.0
        self.head_yaw = 0.0

        # Defining attributes for panel marker orientation:

        # self.panel_roll = 0.0
        # self.panel_pitch = 0.0
        # self.panel_yaw = 0.0

        self.cv = 0.0
        self.auto = 0.0

        # Creating a subscriber that retrieves pose messages published from the aruco marker node:

        self.pose_sub = self.create_subscription(Pose, 'aruco_poses', self.pose_callback, 10)

        # Creating a subscriber that retrieves pose messages published from the panel marker node:

        self.panel_marker_sub = self.create_subscription(Marker, 'glass_panel_rviz', self.panel_marker_callback, 10)
        # Creating a subscriber that recieves messages from joystick:

        self.joystcik_sub = self.create_subscription(Joy, 'joy', self.getJoystickInput, 10)


        # creating a broadcaster that will publish the (Transformstamoed) base frame transform information in reference to the world (odom) frame:

        self.odom_broadcaster = TransformBroadcaster(self, 10)
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'world'
        self.odom_trans.child_frame_id = 'Base'

        # Defining jointstates and creating a jointstate publisher:

        self.jointstate = JointState()
        self.jointstate.name = ['rotation', 'tilt','extension', 'h_yaw', 'h_roll', 'h_pitch']
        self.pub = self.create_publisher(JointState, 'joint_states', 10)


        # creating transform_listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer , self)

        self.target_frame ="Head"
        self.start_frame = "Base"
        self.get_logger().info(f"{name} is on")

    def pose_callback(self, msg):

        # Retrieving aruco pose message and converting it from quaternion to euler radians:

        pose = msg
        orientation_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z , pose.orientation.w]
        (roll_x, pitch_y, yaw_z) = euler_from_quaternion(orientation_list)

        # Converting euler radians to degrees for easier control:

        self.cv_roll = math.degrees(roll_x)
        self.cv_pitch = math.degrees(pitch_y)
        self.cv_yaw= math.degrees(yaw_z)

        # Adjusting head orientation from openCV to match rviz orientation direction:

        if self.cv_roll > 0:
            self.cv_roll = self.cv_roll - 180
        elif self.cv_roll < 0:
            self.cv_roll = self.cv_roll + 180
        else:
            self.cv_roll = 0

        self.cv_yaw =  self.cv_yaw * (-1)
        self.cv_pitch =  self.cv_pitch * (-1)



    def panel_marker_callback(self, msg):

        # Retrieving panel marker pose message and converting it from quaternion to euler radians:

        panel = msg
        panel_quat = [panel.pose.orientation.x, panel.pose.orientation.y, panel.pose.orientation.z , panel.pose.orientation.w]


        print(f'self.panel_quat:{panel_quat}' )

        

        # (panel_roll_x, panel_pitch_y, panel_yaw_z) = euler_from_quaternion(panel_orientation_list)

        # Converting euler radians to degrees for easier control:

        # self.panel_roll  = math.degrees(panel_roll_x)
        # self.panel_pitch = math.degrees(panel_pitch_y)
        # self.panel_yaw   = math.degrees(panel_yaw_z)

        try:

            head_trans =  self.tf_buffer.lookup_transform(self.target_frame, self.start_frame, rclpy.time.Time())

            trans_quat = [head_trans.transform.rotation.x, head_trans.transform.rotation.y, head_trans.transform.rotation.z , head_trans.transform.rotation.w]
            trans_quat_inv = [head_trans.transform.rotation.x, head_trans.transform.rotation.y, head_trans.transform.rotation.z , (head_trans.transform.rotation.w *-1)]

            needed_rot = tf_trans.quaternion_multiply(panel_quat , trans_quat_inv)

            (t_roll_x, t_pitch_y, t_yaw_z) = euler_from_quaternion(needed_rot)

            # Converting euler radians to degrees for easier control:

            self.trans_roll = math.degrees(t_roll_x)
            self.trans_pitch = math.degrees(t_pitch_y)
            self.trans_yaw= math.degrees(t_yaw_z)

        except TransformException as ex : 
            self.get_logger().info(f"I don't see any transform yet")


    def getJoystickInput(self, msg):

        

        # Assigning joystick output to robot movement attributes:

        self.x += self.speed_factor * msg.axes[6]
        self.y += self.speed_factor * msg.axes[7]
        self.arm_rot += msg.axes[0]
        self.arm_tilt += msg.axes[1]
        self.arm_ext += self.speed_factor * msg.buttons[4]
        self.arm_ext -= self.speed_factor * msg.buttons[5]

        # Setting robot movement constrains:

        if self.arm_tilt >=0:
            self.arm_tilt = 0
        if self.arm_tilt <=-60:
            self.arm_tilt = -60

        if self.arm_ext >= 1.1:
            self.arm_ext = 1.1
        if self.arm_ext <= 0.0:
            self.arm_ext = 0.0


        self.auto = msg.buttons[6] 
        self.cv =   msg.buttons[7] 

        if self.cv > 0.0:

            self.head_roll = self.cv_roll
            self.head_pitch = self.cv_pitch
            self.head_yaw = self.cv_yaw

        elif self.auto > 0.0: 

            self.head_roll = self.trans_roll
            self.head_pitch = self.trans_pitch
            self.head_yaw = self.trans_yaw

        

        self.head_roll  += msg.buttons[0]
        self.head_pitch += msg.buttons[2]
        self.head_yaw   += msg.buttons[1]
        self.head_yaw   -= msg.buttons[3]





            

        # # Setting head rotation constrains:

        # if self.head_roll >=60:
        #     self.head_roll = 60
        # if self.head_roll <=-60:
        #     self.head_roll = -60

        # if self.head_pitch >=60:
        #     self.head_pitch = 60
        # if self.head_pitch <=-60:
        #     self.head_pitch = -60   

        # self.end_eff_trans()




        





    # def end_eff_trans(self):

    #     # Retrieving aruco pose message and converting it from quaternion to euler radians:


    #     try:

    #         head_trans =  self.tf_buffer.lookup_transform(self.target_frame, self.start_frame, rclpy.time.Time())

    #         trans_quat = [head_trans.transform.rotation.x, head_trans.transform.rotation.y, head_trans.transform.rotation.z , head_trans.transform.rotation.w]
    #         trans_quat_inv = [head_trans.transform.rotation.x, head_trans.transform.rotation.y, head_trans.transform.rotation.z , (head_trans.transform.rotation.w *-1)]

    #         # print(f'self.panel_quat:{self.panel_quat}' )
    #         # print(f'trans_quat_inv: {trans_quat_inv}')


    #         needed_rot = tf_trans.quaternion_multiply(self.panel_quat , trans_quat_inv)

    #         # (t_roll_x, t_pitch_y, t_yaw_z) = euler_from_quaternion(needed_rot)

    #         # # Converting euler radians to degrees for easier control:

    #         # self.trans_roll = math.degrees(t_roll_x)
    #         # self.trans_pitch = math.degrees(t_pitch_y)
    #         # self.trans_yaw= math.degrees(t_yaw_z)

        # except TransformException as ex : 
        #     self.get_logger().info(f"I don't see any transform yet")

        # self.get_logger().info(f"current orientation: roll: {self.trans_roll}, pitch: {self.trans_pitch}, yaw: {self.trans_yaw}") 
        
        # self.get_logger().info(f"current orientation: roll: {self.head_roll}, pitch: {self.head_pitch}, yaw: {self.head_yaw}") 


        # self.get_logger().info(f"target orientation: roll: {self.panel_roll}, pitch: {self.panel_pitch}, yaw: {self.panel_yaw}") 

        # self.get_logger().info(f"needed rotation: roll: {self.panel_roll - self.trans_roll}, pitch: {self.panel_yaw - self.trans_pitch}, yaw: {self.panel_yaw - self.trans_yaw}") 

        self.broadcastTransformations()

        

            



        





    def broadcastTransformations(self):

        # Updating header stamp transformstamped message: 

        self.time_now = self.get_clock().now().to_msg()
        self.odom_trans.header.stamp = self.time_now

        # Passing robot base movements values to transormstamped attributes: 

        self.odom_trans.transform.translation.x = self.x
        self.odom_trans.transform.translation.y = self.y
        self.odom_trans.transform.translation.z = 0.0

        # Setting the transformstamoed rotation to zero: 

        q= tf_trans.quaternion_from_euler(0.0, 0.0, 0.0)
        self.odom_trans.transform.rotation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

        # Publishing transformstamped message:

        self.odom_broadcaster.sendTransform(self.odom_trans)

        # Updating header stamp joinstate message: 

        self.jointstate.header.stamp = self.time_now

        # Passing robot movement attributes to jointstates in degrees: 

        self.jointstate.position = [np.radians(self.arm_rot), np.radians(self.arm_tilt), self.arm_ext, np.radians(self.head_yaw), np.radians(self.head_roll),  np.radians(self.head_pitch)]

        # Checking if the jointstate positions values match the number of defined jointstates before publishing joinstate positions for rviz: 

        if len (self.jointstate.name) == len (self.jointstate.position):
            self.pub.publish(self.jointstate)
        else:
            print('check position')





def main(args=None):
    rclpy.init(args=args)
    robot = MyRobot('robot')
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        print(f"quitting the program...")
        robot.destroy_node()
    exit(0)


if __name__=='__main__':
    main()