#!/usr/bin/env python

# set 2 goal points at the both sides of the room, 
# let the robot travel around these two points before finding a QR marker.
# when the robot moving, the estimated posiiton 
# from /visp_auto_tracker/object_position is not accurate.
# so after finding a new QR marker, let the robot stop for 2 seconds
# and then send TF transform, get the position of QR marker regarding the map,
# calculate position of the next QR marker, and set the position as goal

import rospy
import actionlib
import tf
import tf_conversions
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist, PoseStamped
import geometry_msgs.msg
import math
import copy

print "Before" 
waypoints = [
    [(-4.5, 0.0, 0.0), (0, 0, 0, 1)],
    [(5.5, 0.0, 0.0), (0, 0, 0, 1)]
]
 
 
def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3] 

    return goal_pose

def qr_found(msg):
  global qr_status
  qr_status = msg.data
  #print(qr_status)

global qr_status
qr_status = 0

def qr_code_scan(msg):
  global X_QR, Y_QR, X_next, Y_next, N, N_next, L, found_cnt
  lines = msg.data.split("\r\n")
  if len(lines) == 6:
    code_qr = []
    for line in lines:
      code_qr.append(line.split("="))

    N = int(code_qr[4][1]) - 1
    N_next = N + 1
    if N_next>4:
      N_next = 0

    if L[N] == '':
      print('found new QR')
      found_cnt +=1
      X_QR[N] = float(code_qr[0][1])
      Y_QR[N] = float(code_qr[1][1])
      X_next[N] = float(code_qr[2][1])
      Y_next[N] = float(code_qr[3][1])
      L[N] = code_qr[5][1]
      print(N, N_next, X_QR[N], Y_QR[N], X_next[N], Y_next[N], L[N])
      

global X_QR, Y_QR, X_next, Y_next, N, N_next, L, found_cnt
X_QR = [0,0,0,0,0]
Y_QR = [0,0,0,0,0]
X_next = [0,0,0,0,0]
Y_next = [0,0,0,0,0]
N = 0
N_next = 0
L = ['','','','','']
found_cnt = 0

def loc_callback(msg):
  global cam_x, cam_y, cam_z, cam_qx, cam_qy, cam_qz, cam_qw
  #print("entering callback")
  cam_x = msg.pose.position.x
  cam_y = msg.pose.position.y
  cam_z = msg.pose.position.z
  cam_qx = msg.pose.orientation.x
  cam_qy = msg.pose.orientation.y
  cam_qz = msg.pose.orientation.z
  cam_qw = msg.pose.orientation.w

global cam_x, cam_y, cam_z, cam_qx, cam_qy, cam_qz, cam_qw
cam_x=0
cam_y=0
cam_z=0
cam_qx=0
cam_qy=0
cam_qz=0
cam_qw=1

'''
def imageobjectPose_callback(msg):
    listener = tf.TransformListener()
    global objectPose
    objectPose = geometry_msgs.msg.PoseStamped()
    objectPose = msg
    br = tf.TransformBroadcaster()
    br.sendTransform((objectPose.pose.position.x, objectPose.pose.position.y, 0),
                     (objectPose.pose.orientation.x, objectPose.pose.orientation.y, objectPose.pose.orientation.z, objectPose.pose.orientation.w),
                     rospy.Time.now(),
                     "carrot1",
                     "camera_optical_link")

    br.sendTransform((6, 1.2, 0),
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     "carrot2",
                     "carrot1")
    try:
        (trans,rot) = listener.lookupTransform('/map', '/carrot1', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("ERROR")
        return
    print("x: ",trans[0],"Y: ",trans[1],"Z: ", trans[2])
    '''

'''
    try:
        (trans,rot) = listener.lookupTransform('/odom', '/carrot2', rospy.Time(0))

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        cmd_vel_pub.publish(cmd)

        print('tf',trans,rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
        #print("error")
        '''

estimated = 0
X_next_map = [0,0,0,0,0]
Y_next_map = [0,0,0,0,0]

if __name__ == '__main__':
    rospy.init_node('project')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    find_qr = rospy.Subscriber('/visp_auto_tracker/status', Int8, qr_found)
    scan_qr = rospy.Subscriber('/visp_auto_tracker/code_message', String, qr_code_scan)
    obj_pos = rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, loc_callback)

    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    goal = goal_pose(waypoints[0])
    client.send_goal(goal)
    print('travel to waypoint 1')

    while not rospy.is_shutdown():
        if qr_status != 3:
            continue

        if found_cnt and estimated == 0:
            print('wait a second')
            client.cancel_all_goals()
            rospy.sleep(3)
            estimated = 1

            br.sendTransform((cam_x, cam_y, cam_z),
                             (cam_qx, cam_qy, cam_qz, cam_qw),
                             rospy.Time.now(),
                             "carrot1",
                             "camera_optical_link")
            while True:
                try:
                    (trans,rot) = listener.lookupTransform('/map', '/carrot1', rospy.Time(0))
                    print("x: ",trans[0],"Y: ",trans[1],"Z: ", trans[2])
                    X_next_map[N_next] = trans[0] + (Y_next[N] - Y_QR[N])
                    Y_next_map[N_next] = trans[1] - (X_next[N] - X_QR[N])
                    print(X_QR[N], Y_QR[N], X_next[N], Y_next[N])
                    print(X_next_map, Y_next_map)
                    goal_pose = MoveBaseGoal()
                    goal_pose.target_pose.header.frame_id = 'map'
                    goal_pose.target_pose.pose.position.x = X_next_map[N_next]
                    goal_pose.target_pose.pose.position.y = Y_next_map[N_next]
                    goal_pose.target_pose.pose.orientation.w = 1 
                    client.send_goal(goal_pose)
                    client.wait_for_result()
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print ('error')
                    pass

        # stop 2 seconds to wait until the estimated position of the QR marker is stable
        rospy.sleep(1)

