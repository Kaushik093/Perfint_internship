#! /usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler
import math
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node("move_group_python",anonymous=True)
group=moveit_commander.MoveGroupCommander("manipulator")


#Publish to /move_group/display_planned_path topic in Rviz to display trajectory on rviz
display_trajectory_publisher=rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=10)

# DISPLAY TRAJECTORY ON GAZEBO

print("Initial pose of end effector : ")
print(group.get_current_pose())    #Current end effector pose


print("initial end effector pose :  ")
initial= group.get_current_pose().pose

print(group.get_current_joint_values())

print(" init posx",initial.position.x)
print(" init posy",initial.position.y)
print(" init posz",initial.position.z)

pose_target= geometry_msgs.msg.Pose()

# Setting target pose for end effector

q = quaternion_from_euler(0,0,0) 

pose_target.orientation.w=q[3]
pose_target.orientation.x=q[2]
pose_target.orientation.y=q[1]
pose_target.orientation.z=q[0]

pose_target.position.x=0.29
pose_target.position.y=0.15
pose_target.position.z=0.2  

# group.set_joint_value_target({'elbow_joint':2.624413043262649})


# pose_target.position.x=0.34
# pose_target.position.y=0.35
# pose_target.position.z=0.37


# group.allow_replanning(True)

group.set_pose_target(pose_target)

group.set_planner_id("RRTConnectkConfigDefault")  #Choosing the planner algorithm from ompl_controllers.yaml
plan1=group.plan()       #Plan the path
group.go(wait=True)   #Execute the path on gazebo

print("Final end effector pose :")
print(group.get_current_pose())  
print(group.get_current_joint_values())
final=group.get_current_joint_values()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()

