#!/usr/bin/env python3

# python
import sys
import copy
# ros
import rospy

import tf2_ros
import tf2_msgs.msg 
import geometry_msgs.msg 
from geometry_msgs.msg import Pose
from enpm809e_msgs.srv import VacuumGripperControl
from enpm809e_msgs.msg import VacuumGripperState, LogicalCameraImage, PartInfos
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# moveit
import moveit_commander as mc

class Part:
    """Part in the workcell is stored in world frame
       So, we can directly pick up the place from the pose
    """
    def __init__(self, color, pose):
        self.color = color
        self.pose = pose
        
        Manipulation.print_partition()
        rospy.loginfo("PART IN WORKCELL")
        rospy.loginfo(self.color)
        rospy.loginfo(self.pose)
        Manipulation.print_partition()
        
class Order:
    """order for the kitting arm to complete
       Each order reprensent the part with it color,
       and it should be moved to where in what bin
       The pose is in world frame. So, we can just use the pose
       to place the part at the pose
    """
   
    def __init__(self, product):
        self.color = product.color
        self.bin = product.bin
        
        # before storing the product pose, we convert
        # the given pose to world frame and then store it
        trans = TF_TRANSFORM.get_world_transform_for_order(product)
        self.pose = Pose()
        self.pose.position.x = trans.translation.x
        self.pose.position.y = trans.translation.y
        self.pose.position.z = trans.translation.z
        
        self.pose.orientation = trans.rotation
        
        Manipulation.print_partition()
        rospy.loginfo("ORDER")
        rospy.loginfo(self.color)
        rospy.loginfo(self.pose)
        Manipulation.print_partition()
        
class TF_TRANSFORM:
    """
    summary:
    The is a Static class for initializing the tf transforms
    """
    tfBuffer = None
    listener = None

    @staticmethod
    def initialize()-> None:
        """initializes tf Buffer
        """
        TF_TRANSFORM.tfBuffer = tf2_ros.Buffer()
        TF_TRANSFORM.listener = tf2_ros.TransformListener(TF_TRANSFORM.tfBuffer)
        
    def get_world_transform_for_order(product):
        """Gets the world tranformation of the product to be moved to where

        Args:
            product (Order): Each order reprensent the part color to be moved to where

        Returns:
            Transformation: This object contains the transformed Translation and Rotation in world frame
        """
        pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1) 
        
        for i in range(3):
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = product.bin
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "part_to_place"
            t.transform.translation.x = product.pose_in_bin.position.x
            t.transform.translation.y = product.pose_in_bin.position.y
            t.transform.translation.z = product.pose_in_bin.position.z

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0 
            t.transform.rotation.z = 0.0 
            t.transform.rotation.w = 1.0

            tfm = tf2_msgs.msg.TFMessage([t])
            pub_tf.publish(tfm)
            
            rospy.sleep(1)
        
        
        trans = TF_TRANSFORM.tfBuffer.lookup_transform('world',
                                                       t.child_frame_id,
                                                       rospy.Time.now() - rospy.Time(3),
                                                       rospy.Duration(2.0))
        
        return trans.transform

class Manipulation(object):

    def __init__(self, node_name='manipulation_809e', ns='',
                 robot_description='robot_description'):

        self.joint_pulisher = rospy.Publisher(
            "/ariac/kitting/kitting_arm_controller/command",
            JointTrajectory,
            queue_size=100)

        mc.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)
        
        Manipulation.print_msg("Manipulation Node for kitting arm activated...")
        TF_TRANSFORM.initialize()
        
        self.parts_in_workcell = []
        self.orders = []

        # kitting_arm
        # - linear_arm_actuator_joint
        # - shoulder_pan_joint
        # - shoulder_lift_joint
        # - elbow_joint
        # - wrist_1_joint
        # - wrist_2_joint
        # - wrist_3_joint

        # dictionary to store pre-set locations
        self.locations = {}

        name = 'home'
        arm_joints = [0, 0, -1.25, 1.74, -2.66, -1.51, 0]
        self.locations[name] = (arm_joints)

        name = 'test'
        arm_joints = [1, 0, -1.25, 1.74, -2.66, -1.51, 0]
        self.locations[name] = (arm_joints)

        self.robot = mc.RobotCommander(ns + '/' + robot_description, ns)
        self.scene = mc.PlanningSceneInterface(ns)

        moveit_group = mc.MoveGroupCommander(
            'kitting_arm', robot_description=ns + '/' + robot_description, ns=ns)
        
        self.groups = {}
        self.groups['kitting_arm'] = moveit_group
        self._arm_group = self.groups['kitting_arm']
    
    @staticmethod
    def print_msg(msg):
        """prints the message hightlighting in the terminal

        Args:
            msg (str): message to be printed
        """
        Manipulation.print_partition()
        rospy.loginfo(msg)
        Manipulation.print_partition()
        
    @staticmethod
    def print_partition():
        """prints a partition containing "="
        """
        rospy.loginfo("="*31)
        
    def main(self):
        """
        Main function to start the Node core
        """

        msg = rospy.wait_for_message("/part_info", PartInfos)
        self.get_orders(msg)
                
        for order in self.orders:
            rospy.sleep(1)
            self.get_parts_in_workcell()
            part = self.find_part(order.color)
            self.move_part(part.pose, order.pose)

    def execute_move(self):
        """Just a general execute of sequence of steps for moving the arm group
        """
        self._arm_group.go(wait=True)
        self._arm_group.stop()
        self._arm_group.clear_pose_targets()

    def find_part(self, part_color):
        """finds the part color in the workcell

        Args:
            part_color (str): color of the part

        Returns:
            Part: returns the necessary part of the 
            same color from the workcell
        """
        for part in self.parts_in_workcell:
            if part.color == part_color:
                break
            
        return part
        
    def get_parts_in_workcell(self):
        """Generates a list of parts which logical camera see
           all parts are stored with their global world pose
        """
        self.parts_in_workcell = []
        red_id = 0
        green_id = 0
        blue_id = 0
        
        def get_parts_in_camera(camera_id):
            """inner function of get_parts_in_workcell, 
               because it is executed twice for two cameras

            Args:
                camera_id (int): camera id of the logical camera
            """
            nonlocal red_id
            nonlocal green_id
            nonlocal blue_id
            camera = "logical_camera" + "_" + str(camera_id)
            msg_camera_1 = rospy.wait_for_message("/logical_camera/" + camera, LogicalCameraImage)
            
            id = 0
            for part in msg_camera_1.models: 
                color = str(part.type).split("_")[2]
                if color == "red":
                    id = red_id
                    red_id += 1
                elif color == "blue":
                    id = blue_id
                    blue_id += 1
                elif color == "green":
                    id = green_id
                    green_id += 1
                    
                part_name = camera + "_" + part.type + "_" + str(id) + "_" + "frame"
                    
                rospy.sleep(1)
                
                t = TF_TRANSFORM.tfBuffer.lookup_transform('world',
                                                        part_name,
                                                        rospy.Time.now(),
                                                        rospy.Duration(2.0))
                pose = Pose()
                pose.position.x = t.transform.translation.x
                pose.position.y = t.transform.translation.y
                pose.position.z = t.transform.translation.z
                
                pose.orientation = t.transform.rotation
                
                self.parts_in_workcell.append((Part(color, pose)))
        
        get_parts_in_camera(1)
        get_parts_in_camera(2)

    def get_orders(self, msg):
        """process order received from navigation node

        Args:
            msg (PartInfos): PartInfos message type
        """
        rospy.loginfo("Part Info received in manipulation node")
        
        for product in msg.part_infos:
            self.orders.append(Order(product))
        
    def reach_goal(self):
        """
        Give a goal to the end effector to reach
        """
        pose_to_reach = copy.deepcopy(self._arm_group.get_current_pose())
        pose_to_reach.pose.position.x -= 1
        pose_to_reach.pose.position.z += 0.5
        self._arm_group.set_pose_target(pose_to_reach)
        self._arm_group.go()

    def publish_joint_values(self, duration=0.1):
        """
        Publish joint values to the Topic /ariac/kitting/kitting_arm_controller/command
        """
        joint_values = [-1, 0.09, -0.75, 2.02, -2.11, -1.58, 0]

        jt_ur10 = JointTrajectory()
        jt_ur10.joint_names = ['linear_arm_actuator_joint',
                               'shoulder_pan_joint',
                               'shoulder_lift_joint',
                               'elbow_joint',
                               'wrist_1_joint',
                               'wrist_2_joint',
                               'wrist_3_joint']

        jtpt = JointTrajectoryPoint()
        jtpt.positions = [joint_values[0],
                          joint_values[1],
                          joint_values[2],
                          joint_values[3],
                          joint_values[4],
                          joint_values[5],
                          joint_values[6]]
        jtpt.velocities = [1, 1, 1, 1, 1, 1, 1]
        jtpt.accelerations = [1, 1, 1, 1, 1, 1, 1]
        jtpt.time_from_start = rospy.Duration.from_sec(duration)
        jt_ur10.points.append(jtpt)
        self.joint_pulisher.publish(jt_ur10)

    def activate_gripper(self):
        """
        Activate a robot's gripper to grasp objects
        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        try:
            control = rospy.ServiceProxy(
                '/ariac/kitting/arm/gripper/control', VacuumGripperControl)
            result = control(True)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def deactivate_gripper(self):
        """
        Deactivate a robot's gripper to release objects
        Returns:
            bool: Service execution result
        """
        rospy.wait_for_service('/ariac/kitting/arm/gripper/control')
        try:
            control = rospy.ServiceProxy(
                '/ariac/kitting/arm/gripper/control', VacuumGripperControl)
            result = control(False)
            return result.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def is_object_attached(self):
        """
        Check whether an object is attached to the gripper

        Returns:
            bool: True if an object is attached, otherwise false
        """
        status = rospy.wait_for_message(
            '/ariac/kitting/arm/gripper/state', VacuumGripperState)
        return status.attached

    def move_arm_base(self, x):
        """
        Only move the joint linear_arm_actuator_joint to the x coordinate

        Args:
            x (float): x position in the world frame
            
        """
        
        # kitting_arm
        # - linear_arm_actuator_joint
        # - shoulder_pan_joint
        # - shoulder_lift_joint
        # - elbow_joint
        # - wrist_1_joint
        # - wrist_2_joint
        # - wrist_3_joint

        x = -1.5 - x
        # arm_joints = [x, 0, -0.75, 2.12, -3.04, -1.51, 0]
        arm_joints = [x, 0, -1.25, 1.74, -2.66, -1.51, 0]
        self._arm_group.go(arm_joints, wait=True)
        self._arm_group.stop()
        self._arm_group.clear_pose_targets()

    def pickandplace(self):
        """
        Hard coded poses for pick and place
        """
        pickup_pose = Pose()
        pickup_pose.position.x = 0
        pickup_pose.position.y = 0
        pickup_pose.position.z = 0.77

        place_pose = Pose()
        place_pose.position.x = -2
        place_pose.position.y = 0
        place_pose.position.z = 0.77

        self.move_part(pickup_pose, place_pose)

    def test_arm_base(self):
        """
        Testing the arm base moves to the correct world x
        """
        self.move_arm_base(0)
        rospy.sleep(3.0)
        self.move_arm_base(-1)
        rospy.sleep(3.0)
        self.move_arm_base(-2)
        rospy.sleep(3.0)
        self.move_arm_base(-3)

    def pick_up_part(self, pickup_pose):
        """
        Pick up a part given its pose

        Args:
            pickup_pose (geometry_msgs.Pose): Pose of the part in the 
            world frame
        """

        # First: get the arm closer to the part
        self.move_arm_base(pickup_pose.position.x)

        # This configuration keeps the gripper flat (facing down)
        flat_gripper = Pose().orientation
        flat_gripper.x = -0.5
        flat_gripper.y = 0.5
        flat_gripper.z = 0.5
        flat_gripper.w = 0.5

        # position to reach = position of the part
        gripper_position = Pose().position
        gripper_position.x = pickup_pose.position.x
        gripper_position.y = pickup_pose.position.y
        gripper_position.z = pickup_pose.position.z + 0.10

        # combine position + orientation
        above_part_pose = Pose()
        above_part_pose.position = gripper_position
        above_part_pose.orientation = flat_gripper

        self.cartesian_move([above_part_pose])

        # activate gripper
        self.activate_gripper()

        # slowly move down until the part is attached to the gripper
        while not self.is_object_attached():
            pickup_pose = copy.deepcopy(self._arm_group.get_current_pose())
            pickup_pose.pose.position.z -= 0.001
            # self._arm_group.set_pose_target(pickup_pose)
            plan, _ = self._arm_group.compute_cartesian_path(
                [pickup_pose.pose], 0.001, 0.0)
            self._arm_group.execute(plan, wait=True)
            self._arm_group.stop()
            self._arm_group.clear_pose_targets()
            rospy.sleep(0.3)
            

        # once the part is attached, lift the gripper
        self.cartesian_move([above_part_pose])
        
    def place_part(self, place_pose):
        """
        Place a part to the given pose

        Args:
            place_pose (geometry_msgs.Pose): Pose of the part in the
            world frame
        """

        # move the arm closer to the drop pose
        self.move_arm_base(place_pose.position.x)

        # ensure the gripper is facing down
        flat_gripper = Pose().orientation
        flat_gripper.x = -0.5
        flat_gripper.y = 0.5
        flat_gripper.z = 0.5
        flat_gripper.w = 0.5

        # set the position to reach
        gripper_position = Pose().position
        gripper_position.x = place_pose.position.x
        gripper_position.y = place_pose.position.y
        gripper_position.z = place_pose.position.z + 0.20

        # set the pose = position + orientation
        above_bin_pose = Pose()
        above_bin_pose.position = gripper_position
        above_bin_pose.orientation = flat_gripper
        self.cartesian_move([above_bin_pose])

        # get the pose of the gripper and make it move a bit lower
        # before releasing the part
        current_arm_pose = copy.deepcopy(self._arm_group.get_current_pose())
        current_arm_pose.pose.position.z -= 0.02
        
        self.cartesian_move([current_arm_pose.pose])

        # deactivate gripper
        self.deactivate_gripper()

        # move the arm up
        # arm_joints = [current_arm_pose.pose.position.x,
        #               0, -1.25, 1.74, -2.66, -1.51, 0]
        # self._arm_group.go(arm_joints, wait=True)

        # go home
        self.go_home()

    def move_part(self, pickup_pose, place_pose):
        """
        Move a part from one pose to another pose

        Args:
            pickup_pose (geometry_msgs.Pose): Current pose of the part in world frame
            place_pose (geometry_msgs.Pose): Pose of the part in the bin in the world frame

        Returns:
            bool: True
        """
        Manipulation.print_partition()
        rospy.loginfo("move_part parameters")
        rospy.loginfo(pickup_pose)
        rospy.loginfo(place_pose)
        Manipulation.print_partition()

        self.pick_up_part(pickup_pose)
        self.place_part(place_pose)

        return True

    def cartesian_move(self, waypoints):
        """
        Move the robotic arm through waypoints

        Args:
            waypoints (List(geometry_msgs.Pose)): List of waypoints
        """
        self._arm_group.set_pose_reference_frame("world")
        (plan, fraction) = self._arm_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        self._arm_group.execute(plan, wait=True)
        self._arm_group.stop()
        self._arm_group.clear_pose_targets()

    def go_home(self):
        """
        Move the robotic arm to the 'home' preset
        location
        """
        self.goto_preset_location('home')

    def goto_preset_location(self, location_name):
        """
        Move the robotic arm to a pre-set location

        Args:
            location_name (str): Pre-set location
        """
        arm = self.locations[location_name]
        location_pose = self._arm_group.get_current_joint_values()
        location_pose[:] = arm
        self._arm_group.go(location_pose, wait=True)
