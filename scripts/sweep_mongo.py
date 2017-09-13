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

import reachability_db


def load_params_ros():
    # this function loads the relevant parameters from the params server
    params = {}

    params['xs'] = np.arange(
        rosparam.get_param('/x_lim/min'),
        rosparam.get_param('/x_lim/max'),
        rosparam.get_param('/x_lim/step'))
    params['ys'] = np.arange(
        rosparam.get_param('/y_lim/min'),
        rosparam.get_param('/y_lim/max'),
        rosparam.get_param('/y_lim/step'))
    params['zs'] = np.arange(
        rosparam.get_param('/z_lim/min'),
        rosparam.get_param('/z_lim/max'),
        rosparam.get_param('/z_lim/step'))

    params['rolls'] = np.arange(
        eval(rosparam.get_param('/roll_lim/min')),
        eval(rosparam.get_param('/roll_lim/max')),
        eval(rosparam.get_param('/roll_lim/step')))
    params['pitchs'] = np.arange(
        eval(rosparam.get_param('/pitch_lim/min')),
        eval(rosparam.get_param('/pitch_lim/max')),
        eval(rosparam.get_param('/pitch_lim/step')))
    params['yaws'] = np.arange(
        eval(rosparam.get_param('/yaws_lim/min')),
        eval(rosparam.get_param('/yaws_lim/max')),
        eval(rosparam.get_param('/yaws_lim/step')))

    params['planner_time_limit'] = rosparam.get_param('/planner_time_limit')
    params['robot_move_group'] = rosparam.get_param('/robot_move_group')
    params['limits_reference_frame'] = rosparam.get_param('/limits_reference_frame')
    params['end_effector_name'] = rosparam.get_param('/end_effector_name')

    return params


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

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    reach_db = reachability_db.ReachabilityDB()

    USE_IK_ONLY = rosparam.get_param('/use_ik_only')

    # load the parameters from ros params server
    params = load_params_ros()

    xs = params['xs']
    ys = params['ys']
    zs = params['zs']
    rolls = params['rolls']
    pitchs = params['pitchs']
    yaws = params['yaws']

    planner_time_limit = params['planner_time_limit']
    robot_move_group = params['robot_move_group']
    limits_reference_frame = params['limits_reference_frame']
    end_effector_name = params['end_effector_name']

    num_combinations = len(xs) * len(ys) * len(zs) * len(rolls) * len(pitchs) * len(yaws)
    print "Looking over: " + str(num_combinations) + " combinations with max time(s) per sample:  " + str(
        planner_time_limit)
    print "This will take: " + str(num_combinations * planner_time_limit / 60 / 60) + " hours"

    robot_reach_space = RobotReachableSpace(group=robot_move_group, planner_time_limit=planner_time_limit)

    approach_tran_to_ee_tran_matrix = get_approach_to_ee(robot_reach_space.m)

    pose = PoseStamped()
    pose.header.frame_id = limits_reference_frame

    jobs = [None] * num_combinations
    count = 0
    for x in tqdm(xs, desc='x'):
        for y in tqdm(ys, desc='y'):
            for z in tqdm(zs, desc='z'):
                for roll in tqdm(rolls, desc='roll'):
                    for pitch in tqdm(pitchs, desc='pitch'):
                        for yaw in tqdm(yaws, desc='yaws'):
                            jobs[count] = [count, x, y, z, roll, pitch, yaw]
                            count += 1

    for job in tqdm(jobs, desc='jobs'):
        count, x, y, z, roll, pitch, yaw = job

        f = PyKDL.Frame(PyKDL.Rotation.RPY(roll, pitch, yaw), PyKDL.Vector(x, y, z))
        pose.pose = tf_conversions.posemath.toMsg(f)
        pose.pose = barrett_grasp_pose_to_moveit_grasp_pose(robot_reach_space.m, pose.pose,
                                                            approach_tran_to_ee_tran_matrix,
                                                            grasp_frame='/approach_tran')

        reachable = int(robot_reach_space.query_pose(pose, end_effector_name, USE_IK_ONLY))
        reach_db.add(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, reachable=reachable)

    roscpp_shutdown()
