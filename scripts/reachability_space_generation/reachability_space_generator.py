
#!/usr/bin/python

from moveit_commander import RobotCommander
import rospy
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest


class RobotReachableSpace(object):
    """docstring for RobotReachableSpaceIK"""

    def __init__(self, move_group_name, planner_time_limit=0.5):
        # instance variables to compute ik
        self.move_group_name = move_group_name
        self.planner_time_limit = planner_time_limit

        rospy.wait_for_service('compute_ik')
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

        # instance variables to compute moveit plan
        self.robot = RobotCommander()
        rospy.sleep(1)
        self.move_group = self.robot.get_group(self.move_group_name)
        self.move_group.set_planning_time(planner_time_limit)

    def get_plan(self, target_pose, end_effector_name='wrist_roll_link'):
        """
        :param target_pose:  a PoseStamped give the desired position of the endeffector.
        """
        # # m.set_pose_target(pose)
        self.move_group.set_pose_target(target_pose, end_effector_name)
        plan = self.move_group.plan()
        return plan

    def get_ik(self, target_pose, end_effector_link='wrist_roll_link'):
        """
        :param target_pose:  a PoseStamped give the desired position of the endeffector.
        """

        service_request = PositionIKRequest()
        service_request.group_name = self.move_group_name
        service_request.ik_link_name = end_effector_link
        service_request.pose_stamped = target_pose
        service_request.timeout.secs = self.planner_time_limit
        service_request.avoid_collisions = True

        try:
            resp = self.compute_ik(ik_request = service_request)
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def query_pose(self, target_pose, end_effector_name='wrist_roll_link', use_ik_only=True):

        if use_ik_only:
            ik_result = self.get_ik(target_pose, end_effector_name)
            return ik_result.error_code.val == 1
        else:
            plan = self.get_plan(target_pose, end_effector_name)
            return len(plan.joint_trajectory.points) > 0