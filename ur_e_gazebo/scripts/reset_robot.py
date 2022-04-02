#! /usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint


moveit_commander.roscpp_initialize(sys.argv)
traj = JointTrajectory()

rospy.init_node("move_group_python",anonymous=True)
robot=moveit_commander.RobotCommander()
scene=moveit_commander.PlanningSceneInterface()
group=moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher=rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,queue_size=10)

print("Initial pose :")
print(group.get_current_joint_values())
pose_target= geometry_msgs.msg.Pose()
    
q = quaternion_from_euler(1.57,3.14,0)

pose_target.orientation.w=q[3]
pose_target.orientation.x=q[2]
pose_target.orientation.y=q[1]
pose_target.orientation.z=q[0]


#Position
pose_target.position.x=0.8155901549769611
pose_target.position.y=0.00343293836587882
pose_target.position.z= 0.005341675504162921

group.set_planner_id("RRTConnectkConfigDefault")


plan1=group.plan()
group.go(wait=True)
print("FINAL POSE :")
print(group.get_current_pose())
print(group.get_current_joint_values())
rospy.sleep(5)
moveit_commander.roscpp_shutdown()







