#! /usr/bin/env python
import rospy
import sys
import copy
import math
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import JointConstraint , Constraints


path_constraints=Constraints()
moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node("move_group_python",anonymous=True)

#Can be used to plan and execute motion on manipulator
group=moveit_commander.MoveGroupCommander("manipulator")

#Publish to /move_group/display_planned_path topic in Rviz to display trajectory on rviz
display_trajectory_publisher=rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=10)


# DISPLAY TRAJECTORY ON GAZEBO

print("Initial pose of end effector : ")
print(group.get_current_pose())    #Current end effector pose
pose_target= geometry_msgs.msg.Pose()

# List of Algorithms to choose from 
Algorithms=[" ","RRT","RRTConnect","RRTstar","T-RRT","PRM","PRMstar"]

# Position coordinates

Target_pos_x=[0.29,0.31,0.34,0.38,0.405,0.42,0.45,0.47,0.8155]
Target_pos_y=[0.15,0.33,0.35,0.36,0.35,0.415,0.43,0.46,0.0034]
Target_pos_z=[0.2,0.3,0.37,0.42,0.44,0.45,0.46,0.48,0.005]


orientation_euler_roll=[0.02,0,0.523,0.392,0,0,3.14,0,0.392,0]
orientation_euler_pitch=[0.523,0.523,-1.5707,3.14,3.14,0,1.5,0.392,0]
orientation_euler_yaw=[1.5707,1.5707,0.523,0,0.785,0,1.5,0.392,0]

orientation_w=[]
orientation_x=[]
orientation_y=[]
orientation_z=[]


for i in range(8):
    q=quaternion_from_euler(float(orientation_euler_roll[i]),float(orientation_euler_pitch[i]),float(orientation_euler_yaw[i])) 
    orientation_w.append(q[3])
    orientation_x.append(q[2])
    orientation_y.append(q[1])
    orientation_z.append(q[0])

# print ("The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3]))
  

# Euler angles for end effector orientation

def move_to_poses():
    

    print("The available algorithms are : ")
    for i in range(1,len(Algorithms)):
        print(i,":",Algorithms[i])
    
    algo_id=int(input("Enter ID of algorithm to choose  : "))
    group.set_planner_id(Algorithms[algo_id])  
    print("Algorithm chosen is :",Algorithms[algo_id])

          
    Goal_id=int(input("Enter ID of goal pose for TCP (0-8) : "))

    # print(Goal_id)

    pose_target.position.x=Target_pos_x[Goal_id]
    pose_target.position.y=Target_pos_y[Goal_id]
    pose_target.position.z=Target_pos_z[Goal_id]
    
    pose_target.orientation.w=orientation_w[Goal_id]
    pose_target.orientation.x=orientation_x[Goal_id]
    pose_target.orientation.y=orientation_y[Goal_id]
    pose_target.orientation.z=orientation_z[Goal_id]

    path_constraints.joint_constraints.append(JointConstraint(joint_name='shoulder_pan_joint',
                                                        position=0, tolerance_above=math.pi/2,
                                                        tolerance_below=0, weight=1))
    
    path_constraints.joint_constraints.append(JointConstraint(joint_name='shoulder_lift_joint',
                                                            position=0, tolerance_above=0,
                                                            tolerance_below=-1.6, weight=1))
    path_constraints.joint_constraints.append(JointConstraint(joint_name='elbow_joint',
                                                          position=0, tolerance_above=2.53,
                                                          tolerance_below=0, weight=1))                                                                                                                
    group.set_path_constraints(path_constraints)  
       
    group.set_pose_target(pose_target)
    # group.set_goal_joint_tolerance(1)
    # group.retime_trajectory(ref_state_in, traj_in)
    
    plan1=group.plan()       #Plan the path


    group.go(wait=True)   #Execute the path on gazebo
    print("Final shoulder pan : ",group.get_current_joint_values()[0])
    print("Final shoulder lift : ",group.get_current_joint_values()[1])
    print("Final elbow lift : ",group.get_current_joint_values()[2])


    Flag=input("enter y/n if you want to move arm to another pose : ")
    if(Flag=='y'):
        move_to_poses()
    elif(Flag=='n'):
        # rospy.sleep(2)
        moveit_commander.roscpp_shutdown()

        
if __name__ == '__main__':
    move_to_poses()
    


# lift = -142 to -90
# pan =  0 to -180
# elbow = -145 to -55

