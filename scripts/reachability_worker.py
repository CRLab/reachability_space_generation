#!/usr/bin/python

import sys
import os
import numpy as np

import math
import PyKDL
import tf_conversions
import rospy
import rosparam
import rospkg
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState
import tf
from geometry_msgs.msg import Transform, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

from tqdm import tqdm
import cProfile
import StringIO
import pstats
import random
import argparse
import yaml

import reachability_db


class RobotReachableSpace(object):
    """docstring for RobotReachableSpaceIK"""

    def __init__(self, group, planner_time_limit=0.5):
        # instance variables to compute ik
        self.group = group
        self.planner_time_limit = planner_time_limit

        rospy.wait_for_service('compute_ik')
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        # instance variables to compute moveit plan
        self.robot = RobotCommander()
        rospy.sleep(1)
        self.m = self.robot.get_group(group)
        self.m.set_planning_time(planner_time_limit)

    def get_plan(self, target_pose, end_effector_name='gripper_link'):
        """
        :param target_pose:  a PoseStamped give the desired position of the endeffector.
        """
        # # m.set_pose_target(pose)
        self.m.set_pose_target(target_pose, end_effector_name)
        plan = self.m.plan()
        return plan

    def get_ik(self, target_pose, end_effector_link='gripper_link'):
        """
        :param target_pose:  a PoseStamped give the desired position of the endeffector.
        """

        service_request = PositionIKRequest()
        service_request.group_name = self.group
        service_request.ik_link_name = end_effector_link
        service_request.pose_stamped = target_pose
        service_request.timeout.secs = self.planner_time_limit
        service_request.avoid_collisions = True

        try:
            resp = self.compute_ik(ik_request=service_request)
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def query_pose(self, target_pose, end_effector_name='gripper_link', use_ik_only=True):

        if use_ik_only:
            ik_result = self.get_ik(target_pose, end_effector_name)
            return ik_result.error_code.val == 1
        else:
            plan = self.get_plan(target_pose, end_effector_name)
            return len(plan.joint_trajectory.points) > 0


def barrett_grasp_pose_to_moveit_grasp_pose(arm_move_group_commander, approach_tran_pose_in_world,
                                            approach_tran_to_end_effector_tran_matrix,
                                            grasp_frame='/approach_tran'):
    """
    :param arm_move_group_commander: A move_group command from which to get the end effector link.
    :type arm_move_group_commander: moveit_commander.MoveGroupCommander
    """
    # get the matrix representing the approach frame in the world coord system
    graspit_grasp_msg_final_grasp_tran_matrix = tf_conversions.toMatrix(
        tf_conversions.fromMsg(approach_tran_pose_in_world))

    # where in the world is the end effector for this grasp
    actual_ee_pose_matrix = np.dot(graspit_grasp_msg_final_grasp_tran_matrix, approach_tran_to_end_effector_tran_matrix)

    # convert matrix back to msg
    ee_pose_in_world = tf_conversions.toMsg(tf_conversions.fromMatrix(actual_ee_pose_matrix))
    return ee_pose_in_world


def get_approach_to_ee(arm_move_group_commander, grasp_frame='/approach_tran'):
    listener = tf.TransformListener()

    try:
        listener.waitForTransform(grasp_frame, arm_move_group_commander.get_end_effector_link(),
                                  rospy.Time(0), timeout=rospy.Duration(1))
        at_to_ee_tran, at_to_ee_rot = listener.lookupTransform(grasp_frame,
                                                               arm_move_group_commander.get_end_effector_link(),
                                                               rospy.Time())
    except Exception as e:
        rospy.logerr("graspit_grasp_pose_to_moveit_grasp_pose::\n " +
                     "Failed to find transform from %s to %s" % (
                         grasp_frame, arm_move_group_commander.get_end_effector_link()))

    approach_tran_to_end_effector_tran_matrix = tf.TransformerROS().fromTranslationRotation(at_to_ee_tran, at_to_ee_rot)

    return approach_tran_to_end_effector_tran_matrix


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upload poses to be checked for reachability.')
    parser.add_argument('--CONFIG_ROOT',
                        default="/home/iakinola/ros/reachability_space_ws/src/reachability_space_generation/configs",
                        type=str,
                        help='Directory containing configuration files describing what tasks to upload to mongo')

    parser.add_argument('--CONFIG_FILENAME',
                        default="barrett.yaml",
                        type=str,
                        help='configuration filename describing what tasks to upload to mongo')

    args = parser.parse_args()

    config_filepath = os.path.join(args.CONFIG_ROOT, args.CONFIG_FILENAME)
    config = yaml.load(open(config_filepath))
    for k, v in config.items():
        args.__dict__[k] = v

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    reach_db = reachability_db.ReachabilityDB()

    robot_reach_space = RobotReachableSpace(group=args.robot_move_group,
                                            planner_time_limit=args.planner_time_limit)

    approach_tran_to_ee_tran_matrix = get_approach_to_ee(robot_reach_space.m)

    while True:
        task = reach_db.get_task()

        f = PyKDL.Frame(PyKDL.Rotation.RPY(task["roll"], task["pitch"], task["yaw"]),
                        PyKDL.Vector(task["x"], task["y"], task["z"]))

        approach_ps = PoseStamped()
        approach_ps.header.frame_id = args.limits_reference_frame
        approach_ps.pose = tf_conversions.posemath.toMsg(f)

        ee_ps = PoseStamped()
        ee_ps.header.frame_id = args.limits_reference_frame
        ee_ps.pose = barrett_grasp_pose_to_moveit_grasp_pose(robot_reach_space.m, approach_ps.pose,
                                                            approach_tran_to_ee_tran_matrix,
                                                            grasp_frame='/approach_tran')

        reachable = bool(robot_reach_space.query_pose(ee_ps, args.end_effector_name, args.use_ik_only))
        print "finished  task: " + str(task)
        reach_db.record_task_result(count=task["count"], reachable=reachable)

    roscpp_shutdown()
