#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, acos, sin, cos, pi

def a(ax, ay, bx, by):
    ma = sqrt(ax * ax + ay * ay) + 0.000001
    mb = sqrt(bx * bx + by * by) + 0.000001
    sc = ax * bx + ay * by
    return acos(sc / ma / mb)

def a2(angle, startX, startY, endX, endY):
    x1 = startX * cos(angle) - startY * sin(angle)
    y1 = startX * sin(angle) + startY * cos(angle)
    x2 = startX * cos(-angle) - startY * sin(-angle)
    y2 = startX * sin(-angle) + startY * cos(-angle)
    length1 = sqrt(((x1 - endX)**2)+((y1 - endY)**2))
    length2 = sqrt(((x2 - endX)**2)+((y2 - endY)**2))
    if length1 < length2:
        return angle
    else:
        return -angle

class vel_manipulator:

    def __init__(self):
        pub_topic_name ="/turtle2/cmd_vel"
        self.pub = rospy.Publisher(pub_topic_name, Twist, queue_size=10)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
        rospy.Subscriber("/turtle2/pose", Pose, self.pose_callback2)
        self.msg = Twist()
        self.msg.linear.x = float(rospy.get_param('~velocity'))
        self.pose1 = Pose()
        self.pose2 = Pose()
        self.x = 1.0
        self.y = 0.0
        self.r = rospy.Rate(1)
        self.r.sleep()

        while(not rospy.is_shutdown()):
            diffX = self.pose1.x - self.pose2.x
            diffY = self.pose1.y - self.pose2.y
            angle = a(self.x,self.y, diffX, diffY)
            angle = a2(angle,self.x, self.y, diffX, diffY)
            rospy.logwarn("angle")
            rospy.logwarn(angle)
            rospy.logwarn(angle * 180 / pi)
            self.msg.angular.z = angle
            oldX = self.x
            oldY = self.y
            self.x = oldX * cos(angle) - oldY * sin(angle)
            self.y = oldX * sin(angle) + oldY * cos(angle)
            self.pub.publish(self.msg)
            self.r.sleep()

    def pose_callback(self, msg):
        self.pose1 = msg

    def pose_callback2(self, msg):
        self.pose2 = msg

if __name__ == '__main__':
    node_name ="Turtlesim_Saver"
    rospy.init_node(node_name)
    vel_manipulator()
    rospy.spin()
