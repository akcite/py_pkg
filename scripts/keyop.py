#!/usr/bin/env python

import sys, tty, termios
import rospy
from geometry_msgs.msg import Twist

#####################################################################

max_linear_vel = 0.5
max_angular_vel = 1.0
linear_vel_step = 0.05
angular_vel_step = 0.1

#####################################################################

def getKey():
  fd = sys.stdin.fileno()
  settings = termios.tcgetattr(fd)
  
  try:
    tty.setraw(fd)
    ch1 = sys.stdin.read(1)
    if ch1 == '\x1b':
      ch2 = sys.stdin.read(1)
      ch3 = sys.stdin.read(1)
      ch1 = ch1 + ch2 + ch3
    
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, settings)
    
  return ch1

#####################################################################

def confine_speed(linear_vel, angular_vel):
  lv_abs = abs(linear_vel)
  av_abs = abs(angular_vel)
  
  if lv_abs > max_linear_vel:
    linear_vel = max_linear_vel * (linear_vel / lv_abs)
  
  if av_abs > max_angular_vel:
    angular_vel = max_angular_vel * (angular_vel / av_abs)
  
  epsilon = 1e-5
  if lv_abs < epsilon:
    linear_vel = 0.
  
  if av_abs < epsilon:
    angular_vel = 0.
  
  return linear_vel, angular_vel
  
#####################################################################

def main():
  rospy.init_node('key_teleop')
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  
  linear_vel = 0.
  angular_vel = 0.
  
  while True:
    while(1):
      k = getKey()
      if k != '':
        break
        
    if k == '\x03':
      break
    
    if k == '\x1b[A': #UP
      linear_vel += linear_vel_step

    elif k == '\x1b[B': #DOWN
      linear_vel -= linear_vel_step

    elif k == '\x1b[C': #RIGHT
      angular_vel -= angular_vel_step

    elif k == '\x1b[D': #LEFT
      angular_vel += angular_vel_step

    elif k == ' ':
      linear_vel = 0.
      angular_vel = 0.

    linear_vel, angular_vel = confine_speed(linear_vel, angular_vel)
    
    print('LV: {} AV: {}'.format(linear_vel, angular_vel))
    
    twist = Twist()
    twist.linear.x = linear_vel
    twist.angular.z = angular_vel
    pub.publish(twist)
    
#####################################################################

if __name__=='__main__':
  main()

#####################################################################

