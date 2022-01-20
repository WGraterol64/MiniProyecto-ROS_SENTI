#!/usr/bin/env python
import sys, tty, termios
import rospy
from geometry_msgs.msg import Twist

PI = 3.1415926535897
SPEED = 40
ROTATE_SPEED = 20
# Converting from angles to radians
ANGULAR_SPPED = ROTATE_SPEED * 2 * PI/360
TIME = 1

# Clase para obtener input del teclado..
class _Getch:
	def __call__(self):
		fd = sys.stdin.fileno()
		old_settings = termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch = sys.stdin.read(3)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch

def move(forward, rotate, node, vel_msg):
		# We wont use linear components
		vel_msg.linear.x  = SPEED * forward

		# Checking if our movement is CW or CCW
		vel_msg.angular.z = ANGULAR_SPPED * rotate

		# Setting the current time
		t0 = rospy.Time.now().to_sec()
		t1 = t0

		while t1 - t0 < TIME:
			node.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()

		# Forcing our robot to stop
		vel_msg.linear.x  = 0
		vel_msg.angular.z = 0
		node.publish(vel_msg)

def move_pepper():
	# Starts a new node
	rospy.init_node('pepper_move_controller', anonymous=True)
	velocity_publisher = rospy.Publisher('/pepper/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()
	vel_msg.linear.y  = 0
	vel_msg.linear.z  = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	inkey = _Getch()

	# Get arrow key
	while True:
		k = inkey()
		if k != '': break

	if   k=='\x1b[A':
		print("Move forward") 
		move(1, 0, velocity_publisher, vel_msg)
	elif k=='\x1b[B': 
		print("Move backward")
		move(-1, 0, velocity_publisher, vel_msg)
	elif k=='\x1b[C': 
		print("Turn counterclockwise") 
		move(0, -1, velocity_publisher, vel_msg)
	elif k=='\x1b[D': 
		print("Turn clockwise") 
		move(0, 1, velocity_publisher, vel_msg)
	else: return False

	return True

if __name__=='__main__':
	try:
		# Testing our function
		while move_pepper(): pass
	except rospy.ROSInterruptException:
			pass
