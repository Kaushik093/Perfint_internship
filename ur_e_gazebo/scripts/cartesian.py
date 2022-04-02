#! /usr/bin/env python
import rospy
import sys
import copy
import moveit_commander 
import geometry_msgs.msg
import moveit_msgs.msg 
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import random
from moveit_msgs.msg import JointConstraint , Constraints
import math


rospy.init_node("move_group_python",anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
arm = moveit_commander.MoveGroupCommander('manipulator')
display_trajectory_publisher=rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=10)
pose_target= geometry_msgs.msg.Pose()

q=quaternion_from_euler(0,0,0)

Target_pos_x=[0.29,0.31,0.34,0.38,0.405,0.42,0.45,0.47]
Target_pos_y=[0.15,0.33,0.35,0.36,0.35,0.415,0.43,0.46]
Target_pos_z=[0.2,0.3,0.37,0.42,0.44,0.45,0.46,0.48]

                                                
def cart():

    inital=arm.get_current_pose().pose

    pose_target.orientation.w=q[3]
    pose_target.orientation.x=q[2]
    pose_target.orientation.y=q[1]
    pose_target.orientation.z=q[1]

    waypoints = []
    
   
    pose_target.position.x=Target_pos_x[0]
    pose_target.position.y=Target_pos_y[0]
    pose_target.position.z=Target_pos_z[0]

    waypoints.append(deepcopy(pose_target))

    pose_target.position.x=(Target_pos_x[1] + Target_pos_x[0])/2
    pose_target.position.y=(Target_pos_y[1] + Target_pos_y[0])/2
    pose_target.position.z=(Target_pos_z[1] + Target_pos_z[0])/2

    waypoints.append(deepcopy(pose_target))


    pose_target.position.x=Target_pos_x[1]
    pose_target.position.y=Target_pos_y[1]
    pose_target.position.z=Target_pos_z[1]

    waypoints.append(deepcopy(pose_target))

    pose_target.position.x=(Target_pos_x[1] + Target_pos_x[2])/2
    pose_target.position.y=(Target_pos_y[1] + Target_pos_y[2])/2
    pose_target.position.z=(Target_pos_z[1] + Target_pos_z[2])/2
    
    waypoints.append(deepcopy(pose_target))
   
    pose_target.position.x=Target_pos_x[2]
    pose_target.position.y=Target_pos_y[2]
    pose_target.position.z=Target_pos_z[2]

    waypoints.append(deepcopy(pose_target))

    # arm.set_planner_id("RRTConnectkConfigDefault")
    path_constraints=Constraints()
    path_constraints.joint_constraints.append(JointConstraint(joint_name='shoulder_pan_joint',
                                                            position=0, tolerance_above=math.pi/2,
                                                            tolerance_below=0, weight=1))
    path_constraints.joint_constraints.append(JointConstraint(joint_name='shoulder_lift_joint',
                                                            position=0, tolerance_above=0,
                                                            tolerance_below=-math.pi/2, weight=1))
    # path_constraints.joint_constraints.append(JointConstraint(joint_name='elbow_joint',
    #                                                       position=0, tolerance_above=2.44,
    #                                                       tolerance_below=0, weight=1))                                                            
    arm.set_path_constraints(path_constraints)    

    
  
    for i in range(len(waypoints)):


        print("Current pose for moving to next pose :  "),
        print(arm.get_current_pose())
        arm.set_planner_id("RRTstarkConfigDefault")

        arm.set_start_state_to_current_state()

        plan, fraction = arm.compute_cartesian_path([waypoints[i]], 0.05, 0.0, True)
        
        arm.execute(plan)
        plan1=arm.plan()
        arm.go(wait=True)
        print(i,"path executed")
        
        rospy.sleep(3)
        print(arm.get_current_joint_values())
    # print("Current pose: "),
    # print(arm.get_current_pose())

cart()


# issues with waypoints
# 1.uncontrolled motion similar to joint angles
# shoulder_lift 0 to -90
# was able to control it by restricting shoulder lift joint from [-pi,pi] to [-pi/2 to 0] in ur5e joint limited urdfxacro

# generally all joints are from -pi to +pi
# Tried to execute after restricting motion for 3 joints but robot did not move at all

