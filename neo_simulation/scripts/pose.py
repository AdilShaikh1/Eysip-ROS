#! /usr/bin/env python

import sys
import copy
import rospy
import math
import tf
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointConstraint, Constraints
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def active_cb(extra):
	rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
	rospy.loginfo("Current location" +str(feedback))

def done_cb(status, result):
	if status == 3:
		rospy.loginfo("Goal reached")

	if status == 2 or status == 8:
		rospy.loginfo("Goal cancelled")

	if status == 4:
		rospy.loginfo("Goal aborted")
'''
moveit_commander.roscpp_initialize(sys.argv)


robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("left_arm")
#group1 = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)
# listener = tf.TransformListener()
start_t = time.time()
tol = 0.03
'''

rospy.init_node('move_python_node',anonymous=True)
navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
navclient.wait_for_server()

goal= MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x = 4.012#first table
goal.target_pose.pose.position.y = 6.978
goal.target_pose.pose.position.z = 0.000
goal.target_pose.pose.orientation.x = 0.000
goal.target_pose.pose.orientation.y = 0.000
goal.target_pose.pose.orientation.z = 0.707108079859 #0.700
goal.target_pose.pose.orientation.w = 0.707105482511 #0.714
#second table -1.993 7.011 0.000 0.000, 0.000, 0.705, 0.70

navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
finished = navclient.wait_for_result()

if not finished:
	rospy.loginfo("Action server not available")
else:
	rospy.loginfo(navclient.get_result())

'''

pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = -2 #-2.0464
pose_target.position.y = 0#-0.0687
pose_target.position.z = 1.4#1.2070
pose_target.orientation.x = -0.5#-0.5813
pose_target.orientation.y = 0.5#0.4026
pose_target.orientation.z = 0.5#0.4022 
pose_target.orientation.w = 0.5#0.5816 
print(pose_target)
arm_group.set_goal_tolerance(0.05)
arm_group.set_pose_target(pose_target)
arm_group.set_path_constraints(upright_constraints)
#group.set_num_planning_attempts(40)
arm_group.set_planning_time(20)
rospy.sleep(5)
plan1 = arm_group.plan()
#print(group.get_path_constraints())
rospy.sleep(5)
arm_group.go(wait=True)



goal1= MoveBaseGoal()
goal1.target_pose.header.frame_id = "map"
goal1.target_pose.header.stamp = rospy.Time.now()

goal1.target_pose.pose.position.x = -1.993#first table
goal1.target_pose.pose.position.y = 7.011
goal1.target_pose.pose.position.z = 0.000
goal1.target_pose.pose.orientation.x = 0.000
goal1.target_pose.pose.orientation.y = 0.000
goal1.target_pose.pose.orientation.z = 0.705
goal1.target_pose.pose.orientation.w = 0.7

#second table -1.993 7.011 0.000 0.000, 0.000, 0.705, 0.70

navclient.send_goal(goal1, done_cb, active_cb, feedback_cb)
finished1 = navclient.wait_for_result()

if not finished:
	rospy.loginfo("Action server not available")
else:
	rospy.loginfo(navclient.get_result())
'''
