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
import rosparam, rospkg

import datetime

import yaml

from geometry_msgs.msg import Transform, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest


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

class RobotReachableSpaceIK(object):
    """docstring for RobotReachableSpaceIK"""
    def __init__(self, group, planner_time_limit=0.005):
        self.group = group
        self.planner_time_limit = planner_time_limit

        rospy.wait_for_service('compute_ik')
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    def get_ik(self, target_pose, end_effector_link='gripper_link'):
        """
        :param target_pose:  a PoseStamped give the desired position of the endeffector.
        """
        
        service_request = PositionIKRequest()
        service_request.group_name = self.group
        service_request.ik_link_name = end_effector_link
        service_request.pose_stamped = target_pose
        service_request.timeout.secs= self.planner_time_limit
        service_request.avoid_collisions = False

        try:
            resp = self.compute_ik(ik_request = service_request)
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def query_pose(self, target_pose, end_effector_name='gripper_link'):

        ik_result = self.get_ik(target_pose, end_effector_name)        
        return ik_result.error_code == 1

class RobotReachableSpaceMoveitFullPlan(object):
    """docstring for RobotReachableSpaceMoveitFullPlan"""

    def __init__(self, group, planner_time_limit=0.5):
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

    def query_pose(self, target_pose, end_effector_name='gripper_link'):

        plan = self.get_plan(target_pose, end_effector_name)        
        return len(plan.joint_trajectory.points) > 0


USE_IK_ONLY = True


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

    count = 0

    filename = rospkg.RosPack().get_path('grasp_reachability_planning')
    filename += '/data/reachability_data_'
    fmt = '%Y-%m-%d-%H-%M-%S'
    filename += datetime.datetime.now().strftime(fmt) + '.csv'
    fd = open(filename, 'w')

    if USE_IK_ONLY:
        robot_reach_space = RobotReachableSpaceIK(group=robot_move_group, planner_time_limit=planner_time_limit)
    else:
        robot_reach_space = RobotReachableSpaceMoveitFullPlan(group=robot_move_group, planner_time_limit=planner_time_limit)

    pose = PoseStamped()
    pose.header.frame_id = limits_reference_frame

    for x in xs:
        for y in ys:
            for z in zs:
                for roll in rolls:
                    for pitch in pitchs:
                        for yaw in yaws:
                            f = PyKDL.Frame(PyKDL.Rotation.RPY(roll, pitch, yaw), PyKDL.Vector(x,y,z))
                            pose.pose = tf_conversions.posemath.toMsg(f)                        

                            # # these lines are not needed except for special data collection 
                            # if USE_IK_ONLY:
                            #     # compute ik for the given pose
                            #     ik_result = robot_reach_space.get_ik(pose, robot_move_group, end_effector_name)
                            # if not USE_IK_ONLY:
                            #     # compute full moveit plan
                            #     plan = robot_reach_space.get_plan(pose, end_effector_name)

                            reachable = robot_reach_space.query_pose(pose, end_effector_name)


                            data = str(count) + " " + str(x) + " " + str(y) + " " + str(z) + " " + str(roll) + " " + str(pitch) + " " + str(yaw) + " " + str(reachable) +"\n"
                            fd.write(data)

                            count += 1

                            if (count%10 == 0):
                                print "status: " + str(count) + "/" + str(num_combinations)

                            # if count > 5:
                            #     import IPython
                            #     IPython.embed()
                            #     assert(False)

    fd.close()

    roscpp_shutdown()
    #np.arange(0,len(xs)+len(ys)+len(zs)+len(rs)+len(ps)+len(yaws)),
    # listener = tf.TransformListener()
    # position, quaternion = listener.lookupTransform("/base_link", "/arm", rospy.Time(0))
    # rosrun tf tf_echo /base_link /gripper_link


