#!/usr/bin/python
import gc
import sys
import os

import PyKDL
import tf_conversions
import rospy
import rospkg
from moveit_commander import roscpp_initialize
from geometry_msgs.msg import PoseStamped
import argparse
import yaml
import tf
import math
import time
import numpy as np
import tqdm

import reachability_space_generator
from reachability_analyzer.message_utils import change_end_effector_link


def get_transfrom(reference_frame, target_frame):
    listener = tf.TransformListener()
    try:
        listener.waitForTransform(reference_frame, target_frame,
                                  rospy.Time(0), timeout=rospy.Duration(1))
        translation_rotation = listener.lookupTransform(reference_frame, target_frame,
                                                        rospy.Time())
    except Exception as e:
        rospy.logerr("get_transfrom::\n " +
                     "Failed to find transform from %s to %s" % (
                         reference_frame, target_frame,))
    return translation_rotation


if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    parser = argparse.ArgumentParser(description='Upload poses to be checked for reachability.')
    CONFIG_ROOT_DEFAULT = rospkg.RosPack().get_path('reachability_space_generation') + '/configs'
    parser.add_argument('--CONFIG_ROOT',
                        default=CONFIG_ROOT_DEFAULT,
                        type=str,
                        help='Directory containing configuration files describing what tasks to upload to mongo')

    parser.add_argument('CONFIG_FILENAME',
                        type=str,
                        help='configuration filename describing what tasks to upload to mongo')

    parser.add_argument('NUM_QUERY_POSES',
                        type=int,
                        help='number of query poses to generate')

    args = parser.parse_args()

    config_filepath = os.path.join(args.CONFIG_ROOT, args.CONFIG_FILENAME)
    config = yaml.load(open(config_filepath))
    for k, v in config.items():
        args.__dict__[k] = v

    xs = np.random.uniform(low=args.x_min, high=args.x_max, size=args.NUM_QUERY_POSES)
    ys = np.random.uniform(low=args.y_min, high=args.y_max, size=args.NUM_QUERY_POSES)
    zs = np.random.uniform(low=args.z_min, high=args.z_max, size=args.NUM_QUERY_POSES)
    rolls = np.random.uniform(low=eval(args.roll_min), high=eval(args.roll_max), size=args.NUM_QUERY_POSES)
    pitchs = np.random.uniform(low=eval(args.pitch_min), high=eval(args.pitch_max), size=args.NUM_QUERY_POSES)
    yaws = np.random.uniform(low=eval(args.yaw_min), high=eval(args.yaw_max), size=args.NUM_QUERY_POSES)

    robot_reach_space = reachability_space_generator.RobotReachableSpace(move_group_name=args.robot_move_group,
                                                                         planner_time_limit=args.planner_time_limit)

    old_ee_to_new_ee_translation_rotation = get_transfrom(args.graspit_link_name, args.end_effector_name)

    output_filepath = "random_reachability_query_data_{}.csv".format(time.strftime("y%y_m%m_d%d_h%H_m%M_s%S"))
    out_file = open(output_filepath, 'w')

    for i in tqdm.tqdm(range(args.NUM_QUERY_POSES)):
        f = PyKDL.Frame(PyKDL.Rotation.RPY(rolls[i], pitchs[i], yaws[i]),
                        PyKDL.Vector(xs[i], ys[i], zs[i]))

        approach_ps = PoseStamped()
        approach_ps.header.frame_id = args.limits_reference_frame
        approach_ps.pose = tf_conversions.posemath.toMsg(f)

        ee_ps = PoseStamped()
        ee_ps.header.frame_id = args.limits_reference_frame
        ee_ps.pose = change_end_effector_link(approach_ps.pose, old_ee_to_new_ee_translation_rotation)

        reachable = bool(robot_reach_space.query_pose(ee_ps, args.end_effector_name, args.use_ik_only))
        out_file.write("{} {} {} {} {} {} {} {}\n".format(i, xs[i], ys[i], zs[i],
                                                          rolls[i], pitchs[i], yaws[i], int(reachable)))
        gc.collect()
