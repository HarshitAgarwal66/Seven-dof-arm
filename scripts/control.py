#!/usr/bin/env python3
import rospy,actionlib
import math
import time
from control_msgs.msg import FollowJointTrajectoryActionGoal,FollowJointTrajectoryGoal,FollowJointTrajectoryAction
from gazebo_msgs.srv import GetLinkState,GetLinkStateResponse
from std_msgs.msg import Header,Float64
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from actionlib_msgs.msg import GoalID
from tf.transformations import euler_from_quaternion
from random import random
PI=3.14

class begin:
	def __init__(self):
		self.pitch=0
		self.z=0	
	
	def call(self):
		sub=rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
		sub(link_name= "wrist_pitch_link",reference_frame= "world")
		l=GetLinkStateResponse()
		self.z=l.link_state.pose.position.z
		x=l.link_state.pose.orientation.x
		y=l.link_state.pose.orientation.y
		zz=l.link_state.pose.orientation.z
		w=l.link_state.pose.orientation.w
		obj=[x,y,zz,w]
		(r,self.pitch,y)=euler_from_quaternion(obj)
	
	def start(self):
		rospy.init_node('control')
		#rospy.wait_for_service('/gazebo/get_link_state')
		pub=rospy.Publisher('/seven_dof_arm/seven_dof_arm_joint_controller/command',JointTrajectory,queue_size=20)
		#pub.wait_for_server()
		#self.call()
		#print(self.z)
		#mess=FollowJointTrajectoryActionGoal()
		#mes=FollowJointTrajectoryGoal()
		msg=JointTrajectory()
		hed=Header()
		hed.frame_id=""
		hed.stamp=rospy.Time.now()
		msg.joint_names=["shoulder_pan_joint","shoulder_pitch_joint","elbow_roll_joint","elbow_pitch_joint","wrist_roll_joint","wrist_pitch_joint","gripper_roll_joint"]
		pos=[0.0,0.0,0.0,0,0,0.0,0]
		k=0
		while not rospy.is_shutdown():
			msg.points.append(JointTrajectoryPoint())
			msg.points[0].positions=[random(),0,0,random(),0,0,0]
			#msg.points[0].velocities=[0,0,0,0,0,0,0]
			msg.points[k].time_from_start=rospy.Duration.from_sec(1)
			k+=1
			#for i in range(7):
			#	msg.points[k].positions[i]=i+2
			#for i in range(7):
			#	msg.points[k].velocities[i]=100*i
			#msg.points[k].time_from_start=rospy.Duration.from_sec(1)
			pub.publish(msg)
			print('success')
		rospy.spin()

if __name__=='__main__':
	a=begin()
	a.start()
