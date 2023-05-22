#!/usr/bin/env python3
import rospy
from final_project.msg import Obstacle , Goal
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from math import atan2
import time
import sys

####################################################################################################################

#Initializing all global variables
mc_pub = None
x=0.0
y=0.0
theta=0.0
goal= Point()
goal.x=0
goal.y=0
twist = Twist()
angle_to_goal=0.0
angle_diff=0.0
obs_det=False

####################################################################################################################

#Delay method used to create a delay in the code
def delay(seconds):
    start_time = time.time()
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= seconds:
            break

####################################################################################################################

#Callback function that gets executed when an Odometry message is received on the odom topic
#Odometry topic is used to get the pose of the robot
def newOdom(msg):
    global x ,y ,theta 

    #Getting the x and y coordinates of the robot
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    #Getting the rotational pose of the robot
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])  

    #Calculating the relative position between our robot and the goal , used to decide its path
    inc_x = goal.x - x
    inc_y = goal.y - y
    angle_to_goal = atan2(inc_y, inc_x) 
    global angle_diff 
    angle_diff= angle_to_goal - theta

####################################################################################################################

#Callback function that gets executed when an Obstacle message is received on the obstacle_detection topic
def ob_callBack(msg):
    global goal
    condition = msg.condition
    move_robot_seek(condition) #Calls the method responsible for actuating the robot

####################################################################################################################

#Callback function that gets executed when a Goal message is received on the goal_input topic
def goal_callBack(msg):

    # Split the input into separate values
    values = msg.coordinate.split()
    
    if len(values) == 2:
        try:
            # Assign the input values to variables
            goal.x = float(values[0])
            goal.y = float(values[1])
            rospy.loginfo("New values - x: " +str(goal.x) + " , y: " + str(goal.y))
        except ValueError:
            rospy.logwarn("Invalid input. Please enter numeric values.")
    else:
        rospy.logwarn("Invalid input. Please enter two values separated by a space.")

####################################################################################################################

#Stops the robot when it reaches our goal point
def brake():
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    mc_pub.publish(twist)

####################################################################################################################

#Aligns the robots front so its facing our goal point , making it move in the shortest distance to our goal
def fix_angle():

    twist.linear.x = 0.0
    if(angle_diff >0.0):
        twist.angular.z = 0.3
    elif (angle_diff<0.0):
        twist.angular.z = -0.3  

    mc_pub.publish(twist)

####################################################################################################################

#Rotates the robot so its facing the desired direction , parallel to the y-axis (condition is not here)
def x_aligned(goal,pos):
    twist.linear.x = 0.0 
    if goal>pos :
        twist.angular.z = 0.3  
    elif goal<pos:
        twist.angular.z = -0.3

#Rotates the robot so its facing the desired direction , parallel to the x-axis (condition is not here)
def y_aligned(goal,pos):
    twist.linear.x = 0.0 
    if goal>pos :
        twist.angular.z = -0.3  
    elif goal<pos:
        twist.angular.z = 0.3

####################################################################################################################

#Main method used to control our robot , leading it down the shortest path and avoiding obstacles
def move_robot_seek(condition):
    global angle_diff , obs_det , x , y , goal , theta

    #Defines the angles of desired directions in the case where the x or y coordinate of our robot aligns with our goal
    #This makes it face the target directly and move straight toward it at one of the predefined angles 
    x_diff= 1.570796327 - abs(theta)
    y_diff= 0.0 - abs(theta)
    y_diff2= 3.141592654 - abs(theta)

    #If our robot reaches our goal (x , y) , stop the robot
    if (abs(goal.x- x)<0.1) and (abs(goal.y-y)<0.1):
      brake()

    elif condition == True: #If goal hasnt been reached, and no obstacle is detected 
        
        if(obs_det): #Used to actuate the robot forward to avoid the obstacle , before rechecking the direction and rotating to face the goal
          
            twist.angular.z = 0.0   
            twist.linear.x = 0.3
            mc_pub.publish(twist) #Publish commands that move the robot
            delay(3)
            obs_det=False

        
        if  abs(goal.x - x)< 0.1 and (x_diff > 0.05): #If the goal (x) and the robots x coordinates align, keep rotating until robot aligned with an axis parallel to the y-axis                                          
            x_aligned(goal.y,y)

        elif abs(goal.x - x)< 0.1 and (x_diff < 0.05):#If the robot is facing our goal, move forwards towards it
            twist.angular.z = 0.0 
            twist.linear.x=0.3

        elif (abs(goal.y - y) < 0.1 and (y_diff > 0.05) and (goal.x>x)) or (abs(goal.y - y) < 0.1 and (y_diff2 > 0.1) and (goal.x<x)): #If the goal (y) and the robots y coordinates align, keep rotating until robot aligned with an axis parallel to the x-axis 
            y_aligned(goal.x,x)

        elif abs(goal.y - y) < 0.1 and (y_diff < 0.05): #If the robot is facing our goal, move forwards towards it
            twist.angular.z = 0.0 
            twist.linear.x=0.3

        elif abs(angle_diff) > 0.1: #If the robot is not facing our goal , rotate until it is
            fix_angle()

        elif abs(angle_diff) < 0.1:#If the robot is facing the goal, move forward towards it
            twist.linear.x = 0.3
            twist.angular.z = 0.0

    else: #If an obstacle is detected , stop moving toward it and rotate away from it
        obs_det=True
        twist.linear.x = 0.0
        twist.angular.z = 0.3

    mc_pub.publish(twist) #Publish the commands that move the robot

####################################################################################################################

def motion_controller():
    global mc_pub

#Properly sets up all publishers and subscribers 
    rospy.init_node('motion_controller', anonymous=True)
    mc_pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
    rospy.Subscriber("/odom", Odometry, newOdom)
    rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('obstacle_detection', Obstacle, ob_callBack)
    rospy.Subscriber('goal_input', Goal, goal_callBack)
    rospy.spin()     

if __name__ == '__main__':
    try:
        motion_controller()
    except rospy.ROSInterruptException:
        pass