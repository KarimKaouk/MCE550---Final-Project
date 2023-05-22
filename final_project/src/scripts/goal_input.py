#!/usr/bin/env python3
import rospy
from final_project.msg import Goal


goal_pub = None

#This method is used to contiously take input from the keyboard in order to change our goal (x , y) when needed
def goal_input():

    global goal_pub
    goal = Goal()
    goal_xy = ""
    
    #Properly intiliazes node and sets up the publisher
    rospy.init_node('goal_input', anonymous=True)
    goal_pub = rospy.Publisher('goal_input', Goal, queue_size=10)

    while not rospy.is_shutdown():
        # Take input from the keyboard
        goal_xy = input("Enter values for x and y: ")
        
        # Split the input into separate values
        values = goal_xy.split()
        
        #Check if the inputs are valid before assigning
        if len(values) == 2:
            try:
                # Assign the input values to variables
                x = float(values[0])
                y = float(values[1])
            except ValueError:
                rospy.logwarn("Invalid input. Please enter numeric values.")
        else:
            rospy.logwarn("Invalid input. Please enter two values separated by a space.")

        #Publish the new coordinates of goal (x , y) as a string
        goal_pub.publish(goal_xy)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        goal_input()
    except rospy.ROSInterruptException:
        pass