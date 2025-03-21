#!/usr/bin/env python

from threading import Lock
import math
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

#####################################################################

class Waypoint():
  def __init__(self, x, y, a, v, w, n):
    self.x = x
    self.y = y
    self.a = a
    self.v = v
    self.w = w
    self.n = n

#####################################################################

MAX_W = 0.4
V_STEP_LEN = 0.1
V_UNIT = 0.1

corner_x = 2.
corner_y = -1.
scan_spacing = 0.5
max_linear_v = 0.3
v_step = 1

mutex = Lock()
odom_pose = [0., 0., 0.]

#####################################################################

def OdomCallback(msg):
  mutex.acquire()
  
  odom_pose[0] = msg.pose.pose.position.x
  odom_pose[1] = msg.pose.pose.position.y
  
  qw = msg.pose.pose.orientation.w
  qz = msg.pose.pose.orientation.z
  odom_pose[2] = math.atan2(2. * qw * qz, qw * qw - qz * qz)
  
  mutex.release()
  #print(odom_pose)  

#####################################################################

def SlamCallback(msg):
  mutex.acquire()
  
  odom_pose[0] = msg.pose.position.x
  odom_pose[1] = msg.pose.position.y
  
  qw = msg.pose.orientation.w
  qz = msg.pose.orientation.z
  odom_pose[2] = math.atan2(2. * qw * qz, qw * qw - qz * qz)
  
  mutex.release()
  #print(odom_pose)  

#####################################################################

def EndPoints(pt0, pt1, start_edge):
  dx = pt1[0] - pt0[0]
  dy = pt1[1] - pt0[1]
  
  length = math.hypot(dx, dy)
  dir_x = dx / length
  dir_y = dy / length
  
  moved = 0.
  if start_edge == False:
    moved += 0.5 * scan_spacing
  
  end_pts = []
  while moved <= length:
    curr_xy = [pt0[0] + dir_x * moved, pt0[1] + dir_y * moved]
    end_pts.append(curr_xy)
    #print(curr_xy[0], curr_xy[1])
    
    moved += scan_spacing
    
  return end_pts  

#####################################################################

def MakePaths(end_pts0, end_pts1):
  paths = []
  
  for i in range(0, len(end_pts0)):
    if i < len(end_pts1):
      path = [end_pts0[i], end_pts1[i]]
      paths.append(path)
    
    if i + 1 < len(end_pts0):
      path = [end_pts1[i], end_pts0[i + 1]]
      paths.append(path)
  
  #for path in paths:
    #print(path[0][0], path[0][1], path[1][0], path[1][1])
  
  return paths      

#####################################################################

def GenerateWaypoints(path):
  pt0 = path[0]
  pt1 = path[1]

  dx = pt1[0] - pt0[0]
  dy = pt1[1] - pt0[1]

  length = math.hypot(dx, dy)
  dir_x = dx / length
  dir_y = dy / length
  
  moved = 0.
  
  forward = [Waypoint(pt0[0], pt0[1], 0., 0., 0., 0)]
  backward = [Waypoint(pt1[0], pt1[1], 0., 0., 0., 0)]
  
  wp_seg = []
  
  if length < 2. * V_STEP_LEN:
    moved = length / 2.
    wp = Waypoint(pt0[0] + dir_x * moved, 
                  pt0[1] + dir_y * moved, 
                  0., 
                  V_UNIT * moved / V_STEP_LEN, 
                  0.,
                  0)
    wp_seg = [forward[0], wp, backward[0]]
    
    return wp_seg
  
  count = 1
  move_limit = length - V_STEP_LEN * 0.5
  
  while moved < length:
    moved += V_STEP_LEN
    if moved < move_limit:
      wp = Waypoint(pt0[0] + dir_x * V_STEP_LEN * count, 
                    pt0[1] + dir_y * V_STEP_LEN * count, 
                    0., 
                    V_UNIT * (count if count < v_step else v_step), 
                    0.,
                    0)
      forward.append(wp)
  
    moved += V_STEP_LEN
    if moved < move_limit:
      wp = Waypoint(pt1[0] - dir_x * V_STEP_LEN * count, 
                    pt1[1] - dir_y * V_STEP_LEN * count, 
                    0., 
                    V_UNIT * (count if count < v_step else v_step), 
                    0.,
                    0)
      backward.append(wp)
      
    count += 1
  
  for wp in forward:
    wp_seg.append(wp)
  
  for wp in reversed(backward):
    wp_seg.append(wp)
  
  #if pt0[0] == 0. and pt0[1] == 0.:
    #for wp in wp_seg:
      #print(wp.x, wp.y, wp.a, wp.v, wp.w, wp.n)
  
  return wp_seg

#####################################################################

def PathPlanning(scan_corners):
  end_pts0 = EndPoints(scan_corners[0], scan_corners[1], True)
  end_pts1 = EndPoints(scan_corners[2], scan_corners[3], False)
  
  paths = MakePaths(end_pts0, end_pts1)
  
  wp_segments = []
  for i in range(0, len(paths)):
    wp_segments.append(GenerateWaypoints(paths[i]))

  return wp_segments

#####################################################################

def ComputeVelocity(seg, curr_pose, wps, i, twist):
  twist.linear.x = abs(twist.linear.x)
  
  dx0 = wps[i - 1].x - curr_pose[0]
  dy0 = wps[i - 1].y - curr_pose[1]
  dx1 = wps[i].x - curr_pose[0]
  dy1 = wps[i].y - curr_pose[1]
  
  l0 = math.hypot(dx0, dy0)
  l1 = math.hypot(dx1, dy1)
  twist.linear.x = l1 / (l0 + l1) * twist.linear.x + l0 / (l0 + l1) * wps[i].v
  
  tx = dx1 - 0.9 * dx0
  ty = dy1 - 0.9 * dy0
  tt = math.atan2(ty, tx)
  
  new_sign = 1.
  new_th = curr_pose[2]
  
  aa = tt - new_th
  aa = math.atan2(math.sin(aa), math.cos(aa))
  if abs(aa) > math.pi / 2:
    new_th = math.atan2(math.sin(new_th + math.pi), 
                        math.cos(new_th + math.pi))
    new_sign = -1.
  
  zz = tt - new_th
  zz = math.atan2(math.sin(zz), math.cos(zz))
  if i == 1 and abs(zz) > math.pi / 4:
    twist.linear.x = 0.
    twist.angular.z = MAX_W if zz > 0. else - MAX_W
    return i, twist
  
  if twist.linear.x > 0.1:
    zz *= 10. * twist.linear.x
  twist.angular.z = zz
  
  abs_zz = abs(zz)
  if abs_zz > math.pi / 4:
    if abs_zz > math.pi / 2:
      abs_zz = math.pi / 2
    twist.linear.x *= 2. - abs_zz * 4. / math.pi
  
  if twist.linear.x < V_UNIT:
    twist.linear.x = V_UNIT
  twist.linear.x *= new_sign
  
  #param_angular_limit = 0.015
  #if abs(twist.angular.z) > param_angular_limit:
    #if twist.angular.z > 0.:
      #twist.angular.z = param_angular_limit
    #else:
      #twist.angular.z = - param_angular_limit
  
  print('<{}-{}>  WP0: {:.3f} {:.3f} {:.3f}  WP1: {:.3f} {:.3f} {:.3f}  '\
        'L01: {:.3f}{:.3f}  '\
        'Pose: {:.3f} {:.3f} {:.3f}  Twist: {:.3f} {:.3f}'.format(
          seg, i, wps[i-1].x, wps[i-1].y, wps[i-1].v,
          wps[i].x, wps[i].y, wps[i].v, l0, l1,
          curr_pose[0], curr_pose[1], curr_pose[2], 
          twist.linear.x, twist.angular.z))
  
  if l1 < l0:
    i += 1
    
  if i == len(wps):
    i -= 1
  
  return i, twist

#####################################################################

def PathComplete(curr_pose, wps):
  n = len(wps)
  l0 = math.hypot(curr_pose[0] - wps[n - 2].x, curr_pose[1] - wps[n - 2].y)
  l1 = math.hypot(curr_pose[0] - wps[n - 1].x, curr_pose[1] - wps[n - 1].y)
  
  if l1 < l0:
    return True
  
  return False

#####################################################################

def main():
  global v_step
  
  rospy.init_node('anavi')
  pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  #sub_odom = rospy.Subscriber('odom', Odometry, OdomCallback)
  sub_slam = rospy.Subscriber('slam_out_pose', PoseStamped, SlamCallback)
 
  v_step = int((max_linear_v + 0.0001) / V_UNIT)
  
  scan_corners = []
  scan_corners.append([0., 0.])
  scan_corners.append([0., corner_y])
  scan_corners.append([corner_x, 0.])
  scan_corners.append([corner_x, corner_y])  
  
  wp_segments = PathPlanning(scan_corners)
  
  rate_loop = rospy.Rate(10)
  while not rospy.is_shutdown():
    if len(wp_segments) == 0:
      rate_loop.sleep()
      continue
    
    for seg in range(0, len(wp_segments)):
      wp_seg = wp_segments[seg]
      twist = Twist()
      twist.linear.x = V_UNIT
      twist.angular.z = 0.
      
      path_step = 1
      while not rospy.is_shutdown():
        mutex.acquire()
        curr_pose = odom_pose
        mutex.release()
        
        if PathComplete(curr_pose, wp_seg):
          twist.linear.x = 0.
          twist.angular.z = 0.
          
          pub_twist.publish(twist)
          for i in range(0, 10):
            rate_loop.sleep()
          
          break
        
        path_step, twist = ComputeVelocity(seg, curr_pose, 
                                           wp_seg, path_step, twist)
        
        pub_twist.publish(twist)
        rate_loop.sleep()
    
    wp_segments = []
    rate_loop.sleep()
  
#####################################################################

if __name__=='__main__':
  main()

#####################################################################

