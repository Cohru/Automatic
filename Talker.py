#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
import math
from nav_msgs.msg import Path
 
x = 0
y = 0
yaw = 0
 
goal_x = 0
goal_y = 0
 
v = 1
w = 0
 
kp = 2
 
distance = 1
 
path = Path()
 
i = 0
 
 
def callback(pose):
    global x, y, yaw, w, delta_angle, distance, goal_x, goal_y
 
    T = 0.005
 
    x = pose.x
    y = pose.y
    yaw = pose.theta
 
    v = pose.linear_velocity
    w = pose.angular_velocity
 
    vx = v * math.cos(yaw)
    vy = v * math.sin(yaw)
 
    x = x + T * vx
    y = y + T * vy
    yaw = yaw + T * w
 
    # Calcul de l'angle entre l'angle actuelle et l'angle de l'objectif
    angle_to_goal = math.atan2(goal_y - y, goal_x - x)
 
    delta_angle = yaw - angle_to_goal
 
    if delta_angle < -(math.pi):
    	delta_angle = delta_angle + 2*(math.pi)
    elif delta_angle > math.pi:
        delta_angle = delta_angle - 2*(math.pi)
 
    w = -kp*delta_angle
 
    distance = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)     
 
 
def goal_callback(goal):
    global goal_x, goal_y
    goal_x = goal.x
    goal_y = goal.y
    rospy.loginfo("Nouvel objectif recu : X = %f, Y = %f", goal_x, goal_y)
 
def nav_callback(data):
    global path, goal_x, goal_y, i
    path = data
    i=0
    goal_x = path.poses[i].pose.position.x
    goal_y = path.poses[i].pose.position.y
 
 
 
 
def move_to_position():
    global goal_x, goal_y, w, v, distance, path, i
 
    # Init
    rospy.init_node('move_to_position', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback)
    sub_goal = rospy.Subscriber('/turtle1/goal', Pose, goal_callback)
    sub_path = rospy.Subscriber('/path', Path, nav_callback)
 
 
    # Actualisation des variable (n = n+1)
    #goal_x = x
    #goal_y = y
 
 
    rate = rospy.Rate(10)
    vel_msg = Twist()
 
 
    while not rospy.is_shutdown():
        print("distance = ", distance, "w = ",w)
        if distance > 0.1:
            # Commande de deplacement
            vel_msg.linear.x = v
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
 
 
            # Commande d'angle
            vel_msg.angular.z = w
 
            # Publier la commande
            pub.publish(vel_msg)
        else:
            # La tortue est arrivée
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.angular.z = 0
            pub.publish(vel_msg)
            if (len(path.poses) - i) != 0:
                i+=1
                goal_x = path.poses[i].pose.position.x
                goal_y = path.poses[i].pose.position.y
 
        rate.sleep()
 
 
 
if __name__ == '__main__':
    try:
        move_to_position()
    except rospy.ROSInterruptException:
        pass