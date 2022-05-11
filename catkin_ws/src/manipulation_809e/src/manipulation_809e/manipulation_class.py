#!/usr/bin/env python


# python
import sys
import copy
# ros
import rospy
from geometry_msgs.msg import Pose
from enpm809e_msgs.srv import VacuumGripperControl
from enpm809e_msgs.msg import VacuumGripperState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# moveit
import moveit_commander as mc


class Manipulation(object):

    def __init__(self, node_name='manipulation_809e', ns='',
                 robot_description='robot_description'):

        self.joint_pulisher = rospy.Publisher(
            "/ariac/kitting/kitting_arm_controller/command",
            JointTrajectory,
            queue_size=100)

        mc.roscpp_initialize(sys.argv)
        rospy.init_node(node_name, anonymous=True)

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
        self._arm_group.set_goal_orientation_tolerance = 0.001
        self._arm_group.set_goal_position_tolerance = 0.001

        # rospy.logerr(self.groups['kitting_arm'].get_current_pose())
        # ee_link
        # rospy.logerr(self.groups['kitting_arm'].get_end_effector_link())

    def main(self):
        """
        Main function to start the Node core
        """
        #

        # forward kinematics through joint values publisher
        self.publish_joint_values()

        # forward kinematics with MoveIt
        # self.goto_preset_location("test")

        # inverse kinematics with MoveIt
        # self.reach_goal()

        # pick-and-place
        # self.pickandplace()

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
        x = -1.5 - x
        arm_joints = [x, 0, -1.25, 1.74, -2.66, -1.51, 0]
        self._arm_group.go(arm_joints, wait=True)

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

        # send the gripper to the pose using moveit
        self._arm_group.set_pose_target(above_part_pose)
        self._arm_group.go()

        # activate gripper
        self.activate_gripper()

        # slowly move down until the part is attached to the gripper
        part_is_attached = self.is_object_attached()
        while not part_is_attached:
            pickup_pose = copy.deepcopy(self._arm_group.get_current_pose())
            pickup_pose.pose.position.z -= 0.001
            self._arm_group.set_pose_target(pickup_pose)
            self._arm_group.go()
            part_is_attached = self.is_object_attached()
            # rospy.sleep(0.2)

        # once the part is attached, lift the gripper
        self._arm_group.set_pose_target(above_part_pose)
        self._arm_group.go()

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
        self._arm_group.set_pose_target(above_bin_pose)
        self._arm_group.go()

        # get the pose of the gripper and make it move a bit lower
        # before releasing the part
        current_arm_pose = copy.deepcopy(self._arm_group.get_current_pose())
        current_arm_pose.pose.position.z -= 0.02
        self._arm_group.set_pose_target(current_arm_pose)
        self._arm_group.go()

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
