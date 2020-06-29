#! /usr/bin/env python

import sys
import copy
import rospy
import math
import tf
import time
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointConstraint, Constraints
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_python_node',anonymous=True)

robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")
#group1 = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)



# ----- NAVIGATION STACK ------#
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

# ------- MOVEIT -------#
listener = tf.TransformListener()
start_t = time.time()
tol = 0.01

obj = "object_26"
flag = True
while not rospy.is_shutdown() and flag is True:
    bol = listener.frameExists(obj)
    if bol is True and flag is True:
        trans,rot = listener.lookupTransform("base_footprint",obj,rospy.Time())
        x,y,z = trans
       # print(x)
        flag = False
    elif bol is False and flag is True: 
        print("error no such transform")

#print(trans)

'''
x= -2
y= -0.2
z=1.07
'''
pose_target = geometry_msgs.msg.PoseStamped()
pose_target.header.frame_id = "base_link"
pose_target.pose.position.x = round(x,2) - 0.010
pose_target.pose.position.y = round(y,2) 
pose_target.pose.position.z = round(z,2)  
pose_target.pose.orientation.x = 0.503554079155
pose_target.pose.orientation.y = 0.504745118071
pose_target.pose.orientation.z = 0.496422192237
pose_target.pose.orientation.w =  0.495207696027
group.set_goal_tolerance(tol)
group.set_pose_target(pose_target)
group.set_num_planning_attempts(40)
group.set_planning_time(50)
plan2 = group.go(wait=True)

rospy.sleep(1)

pose_target.pose.position.x = round(x,1) + 0.03
group.set_pose_target(pose_target)
plan3 = group.go(wait=True)

rospy.sleep(1)

pose_target.pose.position.z = round(z,1) + 0.3
group.set_pose_target(pose_target)
plan7 = group.go(wait=True)



'''
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
