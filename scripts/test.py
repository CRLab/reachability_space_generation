#!/usr/bin/env python
import rospy
import graspit_commander
import numpy as np

if __name__ == "__main__":
    rospy.init_node("test_node")
    rospy.sleep(1.0)
    gc = graspit_commander.GraspitCommander()
    gc.loadWorld("fetch_test_world")
    response = gc.planGrasps(search_energy="GUIDED_REACHABLE_POTENTIAL_QUALITY_ENERGY", max_steps = 50000)
    unchecked_for_reachability_grasps = response.grasps
    rospy.loginfo("We have received grasps from Graspit")

    volume_quality_data = [g.volume_quality for g in unchecked_for_reachability_grasps]
    mean_volume_quality = np.mean(np.array(volume_quality_data))
    rospy.loginfo("The mean volumn quality is " + str(mean_volume_quality))
	with open("/home/bo/ros/experiment_data/data/volumn_quality_result.csv", "a") as myfile:
	    myfile.write(str(mean_volume_quality) + '\n')






    

    rospy.loginfo("checking grasps for reachability")

    reachability_client = actionlib.SimpleActionClient('analyze_grasp_action', graspit_msgs.msg.CheckGraspReachabilityAction)
    reachability_client.wait_for_server()

    reachable_grasps = []
    #Grasp needs to be transford by world space coords of object (which is potentially rotated also)
    #Also the rendering needs to take into account the gripper dof and position
    reachable_data = []
    for i, unchecked_grasp in enumerate(unchecked_for_reachability_grasps):
        rospy.loginfo("checking grasps for reachability")

        grasp = graspit_msgs.msg.Grasp()
        grasp.grasp_id = i

        grasp.object_name = model_name
        pre_grasp_pose = geometry_msgs.msg.Pose()
        pre_grasp_pose.position = unchecked_grasp.pose.position
        pre_grasp_pose.orientation = unchecked_grasp.pose.orientation
        grasp.pre_grasp_pose = pre_grasp_pose

        final_grasp_pose = geometry_msgs.msg.Pose()
        final_grasp_pose.position = unchecked_grasp.pose.position
        final_grasp_pose.orientation = unchecked_grasp.pose.orientation
        #print "here mf: \n\n\n\n\n\n\n\n" + str(final_grasp_pose.position.x)
        #final_grasp_pose.position.x = tmp/50.0
        grasp.final_grasp_pose = final_grasp_pose

        grasp.pre_grasp_dof = [0.09] #Maximum cm the gripper can open #copy in from graspit commander
        grasp.final_grasp_dof = unchecked_grasp.dofs #have the gripper close all the way for now

        #this is the message we are sending to reachability analyzer to check for reachability
        goal = graspit_msgs.msg.CheckGraspReachabilityGoal()
        goal.grasp = grasp

        reachability_client.send_goal(goal)    
        reachability_client.wait_for_result()

        reachability_check_result = reachability_client.get_result()
        reachable_data.append(reachability_check_result)

        if reachability_check_result.isPossible:
            reachable_grasps.append(grasp)

    # save reachable_data to file
    reachable_data_filename = "/home/bo/Desktop/reachability_exp/data/reachable_data.csv"
    rospy.loginfo("printing reachable data...")
    # info = str(reachable_data)
    # rospy.loginfo(info)

    with open(reachable_data_filename,'a') as f_handle:
        f_handle.write(str(reachable_data))
    percent = len(reachable_grasps)*1.0 / len(unchecked_for_reachability_grasps)
    rospy.loginfo ("printing success ratio..." + str(percent))
    percet_result = "ENERGY type" +energy_type + ":\t"+ str(percent)

    percentage_result_filename = "/home/bo/Desktop/reachability_exp/data/percentage_result.csv"
    rospy.loginfo("writing percentage result into file...")
    with open(percentage_result_filename,'a') as f_handle:
        f_handle.write(str(percet_result))
