#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import random
import tf
from nav_msgs.msg import Odometry
from turtle import Turtle
import turtlesim.srv
import turtlesim.msg
from astar_algo import *
from geometry_msgs.msg import Pose
from tf.transformations import *
import time

isObstacle = True
is_current_goal = True
MIN_ANGLE = -60.0/180*math.pi
MAX_ANGLE = 60.0/180*math.pi
minimumDistance = 1.5
vel_msg = Twist()
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
robot_location = Pose()
orientation_data =  Pose()
goal_point = Pose()
current_goal = (0,0)
robot_angle = 0
path = []
req_angle = 0.0

def get_euler_angles(quat):
  #quat_array = np.array([quat.x,quat.y,quat.z,quat.w])
  tr = euler_from_quaternion(quat)
  degree = tr[2] * 180 / math.pi
  return adjust_angle(degree)


def adjust_angle(angle):

  if angle < 0:
    angle = (angle + 360) % 360

  if angle > 360:
    angle = (angle) % 360

  return angle

def get_grid_number((x,y)):
  grid_x = x + 9 
  grid_y = y + 10 
  return (grid_x,grid_y)

def get_angle_distance(x,y,theta1,x1,y1):
  trans = np.sqrt((x-x1)**2 + (y-y1)**2)
  
  ang_diff = np.arctan2(y-y1 , x-x1)
  angle = np.degrees(ang_diff)
  rotation1 = adjust_angle(angle - theta1)
  return rotation1,trans

def locationChanged(locationData):
    global robot_location,orientation_data,robot_current_location,robot_orientation
    global current_goal,is_current_goal,robot_angle,isObstacle,path,req_angle

    robot_location = locationData.pose.pose.position
    orientation_data = locationData.pose.pose.orientation
    #robot_current_location = Point_info(robot_location.x,robot_location.y)
    q = (locationData.pose.pose.orientation.x ,
         locationData.pose.pose.orientation.y ,
         locationData.pose.pose.orientation.z ,
         locationData.pose.pose.orientation.w  )

    robot_orientation = 2 * np.arcsin(orientation_data.z)
    robot_angle = get_euler_angles(q)
    cx = current_goal[0]
    cy = current_goal[1]
    x1 = int(locationData.pose.pose.position.x + 9)
    y1 = int(locationData.pose.pose.position.y + 10)

    d = np.sqrt((cx-x1)**2 + (cy-y1)**2)

    if d < 0.001 and isObstacle == False:
        changeDirection()
        current_goal = path[0]
        path.remove(current_goal)

      

def check_for_goal_reached(point1,point2):
    global robot_current_location
    point3 = robot_current_location

    area = abs(((point1.x * (point2.y - point3.y) + point2.x *(point3.y - point1.y) 
    + point3.x * (point1.y - point2.y))) / 2.0)

    threshold = 0.4
    rospy.loginfo("Area calculate %f", area)
    if area < threshold :
       return True
    else:
       return False

def callback(data):
 global isObstacle
 isObstacle = False
 min_index = int(math.ceil((MIN_ANGLE - data.angle_min)/data.angle_increment))
 max_index = int( math.floor((MAX_ANGLE - data.angle_min)/data.angle_increment))
 #rospy.loginfo("Index generated %d %d", min_index , max_index)
 for i in [min_index,max_index]:
   if float(data.ranges[i]) < 1.0:
      #isObstacle = True
      changeDirection()
      break
  
def changeDirection():
    global isObstacle
    global pub
    global vel_msg , current_goal,is_current_goal,req_angle

    rospy.loginfo("Change direction is called ")
    
    vel_msg.linear.x = 0.0;
    
    
    x1 =  robot_location.x
    y1 = robot_location.y
    cx = current_goal[0]
    cy = current_goal[1]
    theta1 = robot_angle
    angle , d = get_angle_distance(cx,cy,theta1,x1,y1)
    rospy.loginfo("Change direction is called ")
    vel_msg.angular.z = angle * 180 / math.pi
    req_angle = angle
    #random.uniform(0,3.14);
   
    pub.publish(vel_msg)
    isObstacle = False
    is_current_goal = False
       

def astar():
    global isObstacle
    global pub
    global current_goal,is_current_goal,path
    
    rospy.init_node('astar', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    #vel_msg.linear.x = abs(1);
    

    sub = rospy.Subscriber("base_scan", LaserScan, callback)
    odomSub = rospy.Subscriber("base_pose_ground_truth", Odometry, locationChanged)
    rate.sleep()
    gx = rospy.get_param('goalx')
    gy = rospy.get_param('goaly')

    path = get_Path(gx,gy)
    
    print("Paths")
    print(path)
    current_goal = path[0]
    path.remove(current_goal)
    current_goal = path[1]
    
    path.remove(current_goal)
    time.sleep(5)
    changeDirection()
    

    while not rospy.is_shutdown():
        
        #if isObstacle == False and 
        if is_current_goal == False or isObstacle == False:
         rospy.loginfo("Condition is still satisfied %r",isObstacle)
         pub.publish(vel_msg)
         vel_msg.angular.z = 0.0;
         vel_msg.linear.x = abs(0.5)
         rate.sleep()
        elif isObstacle == True:
          rospy.loginfo("the new goal is calculated %r",isObstacle)
          changeDirection()
          #isObstacle = True

          current_goal = path[0]
          path.remove(current_goal)
          #is_current_goal = False

if __name__ == '__main__':
    try:
        astar()
    except rospy.ROSInterruptException:
        pass

#astar()