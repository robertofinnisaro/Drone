#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty

class drone:

    def __init__(self):
        self.pub_launch = rospy.Publisher('/drone/launch', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/drone/land', Empty, queue_size=10)
        self.pub_reset = rospy.Publisher('/drone/reset', Empty, queue_size=10)
        self.pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.flight = False
    
    def launch(self):
        self.pub_launch.publish(Empty())
        self.flight = True

    def land(self):
        self.pub_land.publish(Empty())
        self.flight = False

    def reset(self):
        self.pub_reset.publish(Empty())

    def moveLinear(self,vx,vy,vz):
        self.pub_velocity.publish(Twist(Vector3(vx,vy,vz), Vector3(0,0,0)))

    def moveAngular(self,wx,wy,wz):
        self.pub_velocity.publish(Twist(Vector3(0,0,0), Vector3(wx,wy,wz)))

    def hover(self):
        self.moveLinear(0,0,0)