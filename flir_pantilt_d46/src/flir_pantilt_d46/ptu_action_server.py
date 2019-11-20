#!/usr/bin/env python
#import roslib; roslib.load_manifest('ptu46')
import rospy
import flir_pantilt_d46.msg
import actionlib
from sensor_msgs.msg import JointState
import threading
import numpy as np

class PTUControl(object):
	pan      = 0
	tilt     = 0
	pan_vel  = 0
	tilt_vel = 0
	state_lock = threading.Lock()
	preempt_lock = threading.Lock()

	def __init__(self, sequential_moves):
		# setup some parameters
		self.sequential_moves = sequential_moves
		self.tsmin = rospy.get_param('/ptu/min_tilt_speed', 4.0)
		self.tsmax = rospy.get_param('/ptu/max_tilt_speed', 140.0)
		self.psmin = rospy.get_param('/ptu/min_pan_speed' , 4.0)
		self.psmax = rospy.get_param('/ptu/max_pan_speed' , 140.0)
		self.pstep = rospy.get_param('/ptu/pan_step', 0.00089759763795882463)
		self.tstep = rospy.get_param('/ptu/tilt_step', 0.00089759763795882463)
		self.preempted = False

		self.pan_joint_name = rospy.get_param('/ptu/pan_joint_name')
		self.tilt_joint_name = rospy.get_param('/ptu/tilt_joint_name')

		# setup the subscribers and publishers
		rospy.Subscriber('state', JointState, self.cb_ptu_state)
		self.ptu_pub = rospy.Publisher('cmd', JointState, queue_size=1)
		self.as_goto = actionlib.SimpleActionServer('SetPTUState', \
		flir_pantilt_d46.msg.PtuGotoAction, execute_cb=self.cb_goto,auto_start=False)
		self.as_goto.register_preempt_callback(self.preemptCallback)
		self.as_goto.start()
		self.as_reset  = actionlib.SimpleActionServer('ResetPtu', \
		flir_pantilt_d46.msg.PtuResetAction, execute_cb=self.cb_reset,auto_start=False)
		self.as_reset.register_preempt_callback(self.preemptCallback)
		self.as_reset.start()

	def cb_goto(self, msg):
		pan, tilt, pan_vel, tilt_vel = np.radians((msg.pan, msg.tilt, msg.pan_vel, msg.tilt_vel))

		if not pan_vel:
			pan_vel = self.psmax
		if not tilt_vel:
			tilt_vel = self.tsmax

		self.preempted = False

		self._goto(pan, tilt, pan_vel, tilt_vel)

		result = flir_pantilt_d46.msg.PtuGotoResult()
		result.state.position = self._get_state()
		if not self._get_preempt_status():
			self.as_goto.set_succeeded(result)
		else:
			self.as_goto.set_preempted(result)

	def cb_reset(self, msg):
		self.preempted = False
		self._goto(0,0, self.psmax, self.tsmax)
		result = flir_pantilt_d46.msg.PtuResetResult()
		if not self._get_preempt_status():
			self.as_reset.set_succeeded(result)
		else:
			self.as_reset.set_preempted(result)

	def _goto(self, pan, tilt, pan_vel, tilt_vel):
		rospy.loginfo('going to (%s, %s)' % (pan, tilt))
		#msg_out = JointState()
		#msg_out.header.stamp = rospy.Time.now()
		#msg_out.name = [self.pan_joint_name, self.tilt_joint_name]
		#msg_out.position = [pan, tilt]
		#msg_out.velocity = [pan_vel, tilt_vel]
		#self.ptu_pub.publish(msg_out)
		if self.sequential_moves:
			pan_msg = JointState()
			pan_msg.header.stamp = rospy.Time.now()
			pan_msg.name = [self.pan_joint_name]
			pan_msg.position = [pan]
			pan_msg.velocity = [pan_vel]
			self.ptu_pub.publish(pan_msg)

			# wait for it to get there
			wait_rate = rospy.Rate(10)
			while not self._at_goal((pan, self.tilt)) and not rospy.is_shutdown():
				if self._get_preempt_status():
					return
				wait_rate.sleep()

			tilt_msg = JointState()
			tilt_msg.header.stamp = rospy.Time.now()
			tilt_msg.name = [self.tilt_joint_name]
			tilt_msg.position = [tilt]
			tilt_msg.velocity = [tilt_vel]
			self.ptu_pub.publish(tilt_msg)

			# wait for it to get there
			wait_rate = rospy.Rate(10)
			while not self._at_goal((self.pan, tilt)) and not rospy.is_shutdown():
				if self._get_preempt_status():
					break
				wait_rate.sleep()
		else:
			msg_out = JointState()
			msg_out.header.stamp = rospy.Time.now()
			msg_out.name = [self.pan_joint_name, self.tilt_joint_name]
			msg_out.position = [pan, tilt]
			msg_out.velocity = [pan_vel, tilt_vel]
			self.ptu_pub.publish(msg_out)

			# wait for it to get there
			wait_rate = rospy.Rate(10)
			while not self._at_goal((pan, tilt)) and not rospy.is_shutdown():
				if self._get_preempt_status():
					break
				wait_rate.sleep()

	def _at_goal(self, goal):
		return all(np.abs(np.array(goal) - (self.pan, self.tilt)) <= np.degrees((self.pstep, self.tstep)))

	def cb_ptu_state(self, msg):
		self.state_lock.acquire()
		self.pan, self.tilt = msg.position
		self.state_lock.release()

	def _get_state(self):
		self.state_lock.acquire()
		pt = np.degrees((self.pan, self.tilt))
		self.state_lock.release()
		return pt

	def _get_preempt_status(self):
		self.preempt_lock.acquire()
		preempted = self.preempted
		self.preempt_lock.release()
		return preempted

  	def preemptCallback(self):
		self.preempt_lock.acquire()
		self.preempted = True
		self.preempt_lock.release()

if __name__ == '__main__':
	rospy.init_node('ptu46_action_server')
	sequential_moves = rospy.get_param('~sequential_moves', 'false')
	PTUControl(sequential_moves)
	rospy.spin()
