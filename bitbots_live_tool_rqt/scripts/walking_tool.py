#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard and publishing to "cmd_vel"!
---------------------------
Moving around:
   q    w    e
   a    s    d   

q/e: turn left/right
a/d: left/rigth
w/s: forward/back

Velocities increase / decrease with multiple presses.
SHIFT increases with factor 10

CTRL-C to quit




"""

moveBindings = {
		'w':(1,0,0),
		's':(-1,0,0),
		'a':(0,1,0),
		'd':(0,-1,0),
		'q':(0,0,1),
		'e':(0,0,-1),
		'W':(10,0,0),
		'S':(-10,0,0),
		'A':(0,10,0),
		'D':(0,-10,0),
		'Q':(0,0,10),
		'E':(0,0,-10),
	       }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1, latch=True)
	rospy.init_node('teleop_twist_keyboard')

	x_speed_step = 0.01
	y_speed_step = 0.01
	turn_speed_step = 0.01

	x = 0
	y = 0
	th = 0
	status = 0

	try:
		print(msg)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x += moveBindings[key][0] * x_speed_step
				
				x = round(x, 2)
				y += moveBindings[key][1] * y_speed_step
				y = round(y, 2)
				th += moveBindings[key][2] * turn_speed_step			
				th = round(th, 2)
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x; twist.linear.y = y; twist.linear.z = 0
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
			pub.publish(twist)
			sys.stdout.write("\x1b[A")
			sys.stdout.write("\x1b[A")
			sys.stdout.write("\x1b[A")
			print ("x:    " + str(x) + "     \ny:    " + str(y) + "     \nturn: " + str(th) + "     ")

	except Exception as e:
		print(e)

	finally:
		print("\n")
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
