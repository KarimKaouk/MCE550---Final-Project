#!/usr/bin/env python3
import rospy
from final_project.msg import Obstacle
from sensor_msgs.msg import LaserScan, Image
import cv2
import numpy as np

od_pub = None
threshold = 0.7


#Callback function that gets executed when a LaserScan message is received on the scan topic
def od_callBack(msg):
    global od_pub
    obstacle = Obstacle()
    
    #Retrieving the range values 
    ranges = msg.ranges
    condition = True
    
    #Chhecking for obstacles between 20 and -20 degress infront of the robot
    for i in range(40):
        if ranges[i-20] < threshold:
            condition = False

    #Publish false if obstacle detected , true otherwise
    obstacle.condition = condition
    od_pub.publish(obstacle)


def obstacle_detector():
    global od_pub

#Properly sets up all publishers and subscribers 
    rospy.init_node('osbtacle_detector', anonymous=True)
    rospy.Subscriber('scan', LaserScan, od_callBack)
    od_pub = rospy.Publisher('obstacle_detection', Obstacle, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_detector()
    except rospy.ROSInterruptException:
        pass