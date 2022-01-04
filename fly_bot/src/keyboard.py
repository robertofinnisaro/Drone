#! /usr/bin/env python

import rospy
import math
import sys
import select
import os
from std_msgs.msg import Empty, Bool, Float64MultiArray
from geometry_msgs.msg import Twist, Vector3
from drone import drone
from fly_bot.msg import pilot
from nav_msgs.msg import Odometry

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

def main(drone):
    global navigate
    global record
    global point_array
    global orient_array
    global no_of_points
    global speed
    global mode
    running = True
    rate = rospy.Rate(100)
    record = False
    point_array = ()
    orient_array = ()
    no_of_points = 0
    print("Space - Auto")
    while running:
        key = get_key()
        if key == ' ': #take-off
            drone.launch()
            print("take-off")
            if navigate == True:
                stopper = pilot(True,setpoint,setorient,1,mode)
                stoppilotpub.publish(stopper)
                navigate = False
                print("manually stopped autopilot")
        elif key == '\r': #land
            drone.land()
            print("landing")
            if navigate == True:
                stopper = pilot(True,setpoint,setorient,1,mode)
                stoppilotpub.publish(stopper)
                print("manually stopped autopilot")
                navigate = False
        elif key == 'c':
            break
        if drone.flight:
            if navigate == False:
                navigate = True
                autopilot_start_p = pilot(False,(0,0,0.75,1.5),(0.0,0.0,-20*math.pi/180),1,mode)
                startpilotpub.publish(autopilot_start_p)
                print("Starting - 0,0,0.75,1.5")
        rate.sleep()
    stopper = pilot(True,setpoint,setorient,1,mode)
    if navigate == True:
        stoppilotpub.publish(stopper)
        navigate = False
        print("stopping autopilot")

def get_key():
    if os.name == 'nt':
        if sys.version_info[0] >= 3:
            return msvcrt.getch().decode()
        else:
            return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('master_mover',anonymous=True)
    startpilotpub = rospy.Publisher('/autopilot_start_cmd',pilot,queue_size=10)
    stoppilotpub = rospy.Publisher('/autopilot_stop_cmd',pilot, queue_size=10)
    drone = drone()
    speed = 5               
    navigate = False
    setpoint = (1.0,0.5,1.0) 
    setorient = (0.0,0.0,30*math.pi/180)
    mode_list = ('move+turn','no_turn','reach&turn','turn&move')
    mode = mode_list[0]
    print("Starting Keyboard Controller...")
    main(drone)