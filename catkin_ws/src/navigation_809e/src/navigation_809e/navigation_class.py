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
        
        
        # self.start_aruco_detect()
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
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = -4.18
        goal.target_pose.pose.position.y = 4.26
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0.17
        goal.target_pose.pose.orientation.w = 0.98

        self.client.send_goal(goal,
                              self.done_cb,
                              self.active_cb,
                              self.feedback_cb)

        rospy.spin()

    def active_cb(self):
        rospy.loginfo(
            "Goal pose is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        rospy.loginfo("Moving towards target 1")

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
