#!/usr/bin/env python3


"""I tired on ROS but I found the Pygame approach to be a better approach as Gazebo physics engine wouldn't accept
the Infinte accelaration parameter Hence this is just an approach It's not the solution to any of the tasks"""
import random
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def get_points(num_points):
    points = [[0,0]]
    for i in range(num_points - 1):
        # Generate a random angle in radians
        angle = random.uniform(0, 2 * math.pi)
        
        # Calculate the x and y coordinates of the next point
        m_x = points[-1][0] + math.cos(angle)
        m_y = points[-1][1] + math.sin(angle)
        
        # Add the new point to the array
        points.append([m_x, m_y])
    return points

def euclidean_distance(x1,y1,x2,y2):
    return math.sqrt(math.pow(x1-x2,2)+math.pow(y1-y2,2))


def go_to_next_wp(x_val,y_val,theta_val, point_idx):
    vel_obj = Twist()
    vel_x = 0.1
    vel_z = 0.3

    global points_idx
    distance_diff = euclidean_distance(x_val, y_val, points[points_idx][0], points[points_idx][1])
    if(abs(distance_diff)>tolerance):
        velocity = Twist()
        velocity.linear.x = vel_x
        velocity.linear.y = 0
        velocity.angular.z = vel_z
        vel_pub.publish(velocity)
        


    elif(abs(x_val-points[points_idx][0])<=tolerance and abs(y_val-points[points_idx][1])<=tolerance):
        points_idx+=1

def odom_cb(msg, points_idx):
    x_val = msg.pose.pose.position.x 
    y_val = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    theta_val = euler_from_quaternion(orientation_list)[2]
    global odom_cb_count
    odom_cb_count += 1
    go_to_next_wp(x_val,y_val,theta_val, points_idx)

if __name__ == '__main__':
    #points = {1:(5,0),2:(5,5),3:(0,5),4:(0,0),5:(-5,-5)}
    #points = get_points(100)
    points = [[10,10],[1,1],[2,2],[3,3],[4,4]]
    points_idx = 0
    tolerance = 0.1
    rospy.init_node('wp_node',anonymous=True)
    odom_cb_count = 0
    vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rospy.Subscriber('/odom',Odometry,odom_cb,queue_size=10,callback_args=points_idx)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rospy.spin()

