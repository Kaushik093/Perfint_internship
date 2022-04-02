#! /usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node("move_group_python",anonymous=True)
robot=moveit_commander.RobotCommander()    #Interface robot
scene=moveit_commander.PlanningSceneInterface()    #interface world around robot

#Can be used to plan and execute motion on manipulator
group=moveit_commander.MoveGroupCommander("manipulator")


#Publish to /move_group/display_planned_path topic in Rviz to display trajectory on rviz
display_trajectory_publisher=rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=10)


# DISPLAY TRAJECTORY ON GAZEBO

print("Initial pose of end effector : ")
print(group.get_current_pose())    #Current end effector pose
pose_target= geometry_msgs.msg.Pose()


# Setting target pose for end effector


def move_to_pose():

    
    pose_target.orientation.w=float(input("Enter orientation (w): "))
    pose_target.position.x=float(input("Enter position(x): "))
    pose_target.position.y=float(input("Enter position(y): "))
    pose_target.position.z=float(input("Enter position(z): "))

    group.set_pose_target(pose_target)
    group.set_planner_id("RRTkConfigDefault")  #Choosing the planner algorithm from ompl_controllers.yaml
    plan1=group.plan()       #Plan the path
    group.go(wait=True)   #Execute the path on gazebo

    Flag=input("enter Y/N if you want to move arm to another pose : ")
    if(Flag=='Y'):
        move_to_pose()
    elif(Flag=='N'):
        rospy.sleep(5)
        moveit_commander.roscpp_shutdown()

    
if __name__ == '__main__':
    move_to_pose()







