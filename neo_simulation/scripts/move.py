#! /usr/bin/env python

import sys
import copy
import rospy
import math
import tf
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointConstraint, Constraints

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_python_node',anonymous=True)

robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)
#listener = tf.TransformListener()
#flag = True
'''
while not rospy.is_shutdown() and flag is True:
    bol = listener.frameExists("object_1")
    if bol is True and flag is True:
        trans,rot = listener.lookupTransform("world","object_1",rospy.Time())
        bol = listener.frameExists("object_1")
        x,y,z = trans
        tx,ty,tz,w = rot 
        flag = False
        print(trans)
       '''
    #^^ 
'''
upright_constraints = Constraints()
joint_constraint = JointConstraint()
upright_constraints.name = "upright"
joint_constraint.position = -1.60
joint_constraint.tolerance_above = 1.4
joint_constraint.tolerance_below = 1.61
joint_constraint.weight = 1
joint_constraint.joint_name = "link2_link3_joint"
upright_constraints.joint_constraints.append(joint_constraint)
3.888063 7.965505 1.015001
'''

pose_target = geometry_msgs.msg.PoseStamped()
pose_target.header.frame_id = "base_link"
pose_target.pose.position.x = 0.737110121358 #0.766042118377
pose_target.pose.position.y = 0.0701115258766 #0.0199700147451
pose_target.pose.position.z = 1.00626186584 #0.767600195051
pose_target.pose.orientation.x = 0.503554079155
pose_target.pose.orientation.y = 0.504745118071
pose_target.pose.orientation.z = 0.496422192237
pose_target.pose.orientation.w =  0.495207696027
#print(pose_target)
group.set_goal_tolerance(0.01)
group.set_pose_target(pose_target)
#group.set_path_constraints(upright_constraints)
group.set_num_planning_attempts(40)
group.set_planning_time(50)
#rospy.sleep(2)
plan2 = group.plan()
'''
while not rospy.is_shutdown():
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
'''
#print(group.get_path_constraints())
#rospy.sleep(5)
group.go(wait=True)

#rospy.sleep(5) 
print(group.get_current_pose())

moveit_commander.roscpp_shutdown()
