#!/usr/bin/env python3

import rosnode
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, TransformStamped
from fiducial_msgs.msg import FiducialTransformArray
import tf2_ros
import rosnode
from enpm809e_msgs.msg import PartInfo, PartInfos

class Navigation(object):
    """
    A controller class to drive a mobile base in Gazebo.
    """
    DATA_PUBLISHED = False

    def __init__(self, rate=10):
        rospy.init_node('navigation_809e', anonymous=False)
        
        rospy.loginfo("="*21)
        rospy.loginfo("Navigation Node for Turtlebot activated...")
        rospy.loginfo("="*21)
        
        rospy.loginfo('Press Ctrl c to exit')
        
        self.fiducial_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_tranform_detected_cb)
        self.fiducial_sub_unregistered = False
      
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
        
        self.part_info_dic = {}
        
        self.movebase_client()
        
        
    def publish_part_info_data(self):
        """This Published the part info message containing
           information of two orders from two aruco markers.
           The data is sent only once, because the manipulation
           node only need the data once.
        """
        
        if not Navigation.DATA_PUBLISHED:
            Navigation.DATA_PUBLISHED = True
            
            part_infos_msg = PartInfos()
            part_info_list = []
            
            for part_info in self.part_info_dic.values():
                part_info_list.append(part_info)
            
            part_infos_msg.part_infos = part_info_list
            rospy.loginfo(part_info_list)
            
            self._part_infos_pub.publish(part_infos_msg)
            
            rospy.loginfo("PARTINFO DATA PUBLISHED")
            
            # kills aruco detect
            rosnode.kill_nodes(["aruco_detect"])
        
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
   
    def get_goal_for_target(self, i):
        """This gives the MoveBaseGoal object for the movebase client

        Args:
            i (int): The Target Id i.e. 1, 2 and 3

        Returns:
            MoveBaseGoal: the MoveBaseGoal object for its corresponding target id
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/position_x')
        goal.target_pose.pose.position.y = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/position_y')
        goal.target_pose.pose.orientation.x = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/orientation_x')
        goal.target_pose.pose.orientation.y = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/orientation_y')
        goal.target_pose.pose.orientation.z = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/orientation_z')
        goal.target_pose.pose.orientation.w = rospy.get_param('/aruco_lookup_locations/target_'+str(i)+'/orientation_w')
        return goal
        

    def movebase_client(self):
        """Starts the movebase client for target 1
        """
        goal = self.get_goal_for_target(1)
    
        self.client.send_goal(goal,
                            self.done_cb,
                            self.active_cb,
                            self.feedback_cb)
        
        rospy.spin()

    def fiducial_tranform_detected_cb(self, data):
        """The callback function for topic /fiducial_transforms

        Args:
            data (FiducialTransformArray): The data received from topic /fiducial_transforms 
        """
        if not self.fiducial_sub_unregistered:
            if data.transforms:
                fiducial_id = str(data.transforms[0].fiducial_id)
                part_info_msg = PartInfo()
                part_info_msg.bin = rospy.get_param("/kits/aruco_"+ fiducial_id +"/bin")
                part_info_msg.color = rospy.get_param("/kits/aruco_"+ fiducial_id +"/part/color")
                part_info_msg.pose_in_bin = Pose()
                part_info_msg.pose_in_bin.position.x = rospy.get_param("/kits/aruco_"+ fiducial_id +"/part/location/position_x")
                part_info_msg.pose_in_bin.position.y = rospy.get_param("/kits/aruco_"+ fiducial_id +"/part/location/position_y")
                part_info_msg.pose_in_bin.position.z = rospy.get_param("/kits/aruco_"+ fiducial_id +"/part/location/position_z")
                self.part_info_dic.update({fiducial_id:part_info_msg})
                if len(self.part_info_dic) == 2:
                    self.publish_part_info_data()
                    
                    # unsuscribe from fiducial detected callbacks we dont need it anymore
                    self.fiducial_sub.unregister()
                    self.fiducial_sub_unregistered = True
        
    def active_cb(self):
        pass

    def feedback_cb(self,feedback):
        pass
    

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
            goal = self.get_goal_for_target(2)
            self.client.send_goal(goal,
                            self.done_cb2,
                            self.active_cb,
                            self.feedback_cb)

    def done_cb2(self, status, result):
        """
        Callback when movebase has reached the target 2

        Args:
            status (int): status of the execution
            result (str): Resut from the Action Server

        Returns:
            str: Result from the Action Server
        """
            
        if status == 3:
            rospy.loginfo("Goal pose reached")
            goal = self.get_goal_for_target(3)
            self.client.send_goal(goal,
                            self.done_cb3,
                            self.active_cb,
                            self.feedback_cb)
            
    def done_cb3(self, status, result):
        """Callback when its reached target 3,
           We do nothing, when we reach goal point

        Args:
            status (int): status of the execution
            result (str): Resut from the Action Server

        Returns:
            str: Result from the Action Server
        """
        pass
        

