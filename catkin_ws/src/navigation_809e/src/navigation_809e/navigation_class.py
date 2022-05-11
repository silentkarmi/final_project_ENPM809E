#!/usr/bin/env python

import rospy
import sys
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, TransformStamped
from fiducial_msgs.msg import FiducialTransformArray
import copy
import tf2_ros
import rosnode
from enpm809e_msgs.msg import PartInfo, PartInfos

class Navigation(object):
    """
    A controller class to drive a mobile base in Gazebo.
    """

    def __init__(self, rate=10):
        rospy.init_node('navigation_809e', anonymous=False)
        
        rospy.loginfo("="*21)
        rospy.loginfo("Navigation Node for Turtlebot activated...")
        rospy.loginfo("="*21)
        
        rospy.loginfo('Press Ctrl c to exit')
        rospy.Subscriber("/fiducial_transforms",
                         FiducialTransformArray, self.fiducial_transforms_cb)
        self._part_infos_pub = rospy.Publisher(
            '/part_info', PartInfos, queue_size=10, latch=True)

        self.client = actionlib.SimpleActionClient(
            'waffle/move_base', MoveBaseAction)
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        
        # ToDo Pradnya: Remove this Example Code after implemenation of the part message from aruco markers
        part1_info_msg = PartInfo()
        part1_info_msg.bin = "bin4"
        part1_info_msg.color = "red"
        part1_info_msg.pose_in_bin = Pose()
        part1_info_msg.pose_in_bin.position.x = 0.0
        part1_info_msg.pose_in_bin.position.y = 0.0
        part1_info_msg.pose_in_bin.position.z = 0.77
        
        part2_info_msg = PartInfo()
        part2_info_msg.bin = "bin3"
        part2_info_msg.color = "green"
        part2_info_msg.pose_in_bin = Pose()
        part2_info_msg.pose_in_bin.position.x = 0.15
        part2_info_msg.pose_in_bin.position.y = -0.1
        part2_info_msg.pose_in_bin.position.z = 0.77
        
        part_infos_msg = PartInfos()
        part_info_list = [part1_info_msg, part2_info_msg]
        part_infos_msg.part_infos = part_info_list
        
        rospy.sleep(10)
        
        self._part_infos_pub.publish(part_infos_msg)
        
    
        rospy.loginfo("="*21)
        rospy.loginfo("Part Info Published...")
        rospy.loginfo("="*21)
        
        # # self.start_aruco_detect()
        self.movebase_client()
        
        

    def get_transform(self, source, target):
        tf_buffer = tf2_ros.Buffer(rospy.Duration(3.0))
        tf2_ros.TransformListener(tf_buffer)

        transform_stamped = TransformStamped()
        # Get the transform between robot_map and robot_arm_tool0

        for _ in range(5):
            try:
                transform_stamped = tf_buffer.lookup_transform(
                    source,
                    target,
                    rospy.Time(),
                    rospy.Duration(1.0))
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.logerr("Unable to lookup transform")

        pose = Pose()
        pose.position = transform_stamped.transform.translation
        pose.orientation = transform_stamped.transform.rotation
        return pose

    def fiducial_transforms_cb(self, msg):
        pass

    def movebase_client(self):
        goal = MoveBaseGoal()
        # if rospy.has_param('/aruco_lookup_locations/target_1/position_x'):
        #     maybe= rospy.get_param('/aruco_lookup_locations/target_1/position_x')
        #     rospy.loginfo('--------------')
        #     rospy.loginfo(maybe)
        #     rospy.loginfo('--------------')
        # else:
        #     rospy.loginfo('Parameter does not exist!!!!!')

        for i in range(1,4):
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/position_x')
            goal.target_pose.pose.position.y = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/position_y')
            goal.target_pose.pose.orientation.x = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/orientation_x')
            goal.target_pose.pose.orientation.y = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/orientation_y')
            goal.target_pose.pose.orientation.z = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/orientation_z')
            goal.target_pose.pose.orientation.w = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/orientation_w')
            self.client.send_goal(goal,
                                self.done_cb,
                                self.active_cb,
                                self.feedback_cb)
            rospy.loginfo('Target done')
            rospy.loginfo(i)
            rospy.spin()

    def active_cb(self):
        rospy.loginfo(
            "Goal pose is now being processed by the Action Server...")

    def feedback_cb(self,feedback):
        rospy.loginfo('Getting feedback')
    

    def done_cb(self, status, result):
        """
        Callback when movebase has reached the goal

        Args:
            status (int): status of the execution
            result (str): Resut from the Action Server

        Returns:
            str: Result from the Action Server
        """
            
        if status == 3:
            rospy.loginfo("Goal pose reached")


            # rosnode.kill_nodes(["aruco_detect"])
            
            # write code to send the robot to the next target


