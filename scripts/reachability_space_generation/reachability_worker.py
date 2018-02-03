#!/usr/bin/python
import gc
import sys
import os

import PyKDL
import tf_conversions
import rospy
import rospkg
from moveit_commander import RobotCommander, roscpp_initialize
from geometry_msgs.msg import Transform, PoseStamped
import argparse
import yaml
import copy
import tf
import math
import numpy as np

import reachability_db
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

    args = parser.parse_args()

    config_filepath = os.path.join(args.CONFIG_ROOT, args.CONFIG_FILENAME)
    config = yaml.load(open(config_filepath))
    for k, v in config.items():
        args.__dict__[k] = v

    args.rolls = np.arange(eval(args.roll_min), eval(args.roll_max), eval(args.roll_step))
    args.pitchs = np.arange(eval(args.pitch_min), eval(args.pitch_max), eval(args.pitch_step))
    args.yaws = np.arange(eval(args.yaw_min), eval(args.yaw_max), eval(args.yaw_step))

    reach_db = reachability_db.ReachabilityDB(args.incomplete_task_collection_name, args.finished_task_collection_name)

    robot_reach_space = reachability_space_generator.RobotReachableSpace(move_group_name=args.robot_move_group,
                                                                         planner_time_limit=args.planner_time_limit)

    old_ee_to_new_ee_translation_rotation = get_transfrom(args.graspit_link_name, args.end_effector_name)

    while True:
        bulk_task = reach_db.get_bulk_task()
        tasks_to_be_uploaded = []
        cache = dict()
        for roll in args.rolls:
            for pitch in args.pitchs:
                for yaw in args.yaws:
                    task = copy.copy(bulk_task)
                    del task["_id"]
                    del task["status"]
                    task['roll'] = roll
                    task['pitch'] = pitch
                    task['yaw'] = yaw

                    f = PyKDL.Frame(PyKDL.Rotation.RPY(task["roll"], task["pitch"], task["yaw"]),
                                    PyKDL.Vector(task["x"], task["y"], task["z"]))
                    if str(f) in cache:
                        task["reachable"] = cache[str(f)]
                        continue

                    approach_ps = PoseStamped()
                    approach_ps.header.frame_id = args.limits_reference_frame
                    approach_ps.pose = tf_conversions.posemath.toMsg(f)

                    ee_ps = PoseStamped()
                    ee_ps.header.frame_id = args.limits_reference_frame
                    ee_ps.pose = change_end_effector_link(approach_ps.pose, old_ee_to_new_ee_translation_rotation)

                    reachable = bool(robot_reach_space.query_pose(ee_ps, args.end_effector_name, args.use_ik_only))
                    task["reachable"] = reachable
                    cache[str(f)] = reachable

                    tasks_to_be_uploaded.append(task)

        print "finished  bulk task: " + str(bulk_task)
        reach_db.record_task_result(count=bulk_task["count"], tasks_to_be_uploaded=tasks_to_be_uploaded)
        gc.collect()

