#!/usr/bin/python

import sys
import rospy
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState
import numpy as np
import math
import PyKDL
import tf_conversions
import rospy
import rosparam

from datetime import datetime as dt

import yaml

def load_yaml(yaml_file="fetch_params.yaml"):
	with open(yaml_file, 'r') as stream:
		try:
			content = yaml.load(stream)
		except yaml.YAMLError as exc:
			print(exc)
	return content

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
        rosparam.get_param('/roll_lim/min'),
        eval(rosparam.get_param('/roll_lim/max')),
        rosparam.get_param('/roll_lim/step'))
    params['pitchs'] = np.arange(
        eval(rosparam.get_param('/pitch_lim/min')),
        eval(rosparam.get_param('/pitch_lim/max')),
        rosparam.get_param('/pitch_lim/step'))
    params['yaws'] = np.arange(
        rosparam.get_param('/yaws_lim/min'),
        eval(rosparam.get_param('/yaws_lim/max')),
        rosparam.get_param('/yaws_lim/step'))

    params['planner_time_limit'] = rosparam.get_param('/planner_time_limit')
    params['robot_move_group'] = rosparam.get_param('/robot_move_group')
    params['limits_reference_frame'] = rosparam.get_param('/limits_reference_frame')
    params['end_effector_name'] = rosparam.get_param('/end_effector_name')

    return params


if __name__ == '__main__':

	# # load the parameters from the configs file
	# limits_config_file = 'config/fetch_params.yaml'
	# params = load_params(limits_config_file)

    # load the parameters from ros params server
    params = load_params_ros()

    xs = params['xs']
    ys = params['ys']
    zs = params['zs']
    rolls =  params['rolls']
    pitchs =  params['pitchs']
    yaws =  params['yaws']

    planner_time_limit =  params['planner_time_limit']
    robot_move_group =  params['robot_move_group']
    limits_reference_frame =  params['limits_reference_frame']
    end_effector_name = params['end_effector_name']

    num_combinations = len(xs)*len(ys)*len(zs)*len(rolls)*len(pitchs)*len(yaws)
    print "Looking over: " + str(num_combinations) + " combinations with max time(s) per sample:  " + str(planner_time_limit)
    print "This will take: "  + str(num_combinations * planner_time_limit / 60 / 60 )  + " hours"

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    robot = RobotCommander()
    rospy.sleep(1)

    count = 0
    m = robot.get_group(robot_move_group)
    m.set_planning_time(planner_time_limit)

    pose = m.get_current_pose()
    for x in xs:
        for y in ys:
            for z in zs:
                for roll in rolls:
                    for pitch in pitchs:
                        for yaw in yaws:
                            f = PyKDL.Frame(PyKDL.Rotation.RPY(roll, pitch, yaw), PyKDL.Vector(x,y,z))
                            pose.pose = tf_conversions.posemath.toMsg(f)
                            # m.set_pose_target(pose)
                            m.set_pose_target(pose, end_effector_name)
                            plan = m.plan()

                            reachable = True
                            if len(plan.joint_trajectory.points) == 0:
                                reachable = False

                            fd = open("reachability_data_"+dt.now().strftime("%d_%m_%y_%I_%M%p")+".csv", 'a')
                            data = str(count) + " " + str(x) + " " + str(y) + " " + str(z) + " " + str(roll) + " " + str(pitch) + " " + str(yaw) + " " + str(reachable) +"\n"
                            fd.write(data)
                            fd.close()

                            count += 1

                            if (count%10 == 0):
                                print "status: " + str(count) + "/" + str(num_combinations)

                            # if count > 5:
                            #     import IPython
                            #     IPython.embed()
                            #     assert(False)



    roscpp_shutdown()
    #np.arange(0,len(xs)+len(ys)+len(zs)+len(rs)+len(ps)+len(yaws)),
    # listener = tf.TransformListener()
    # position, quaternion = listener.lookupTransform("/base_link", "/arm", rospy.Time(0))
    # rosrun tf tf_echo /base_link /gripper_link

    # TODO: fill configs files
    # write up launch file
