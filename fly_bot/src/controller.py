#! /usr/bin/env python

import rospy
import math
import time
import tf
import numpy as np
import csv
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from fly_bot.msg import pilot
from drone import drone

class PID:

    def __init__(self,kp=[0.5,0.5,0.5,0.5],kd=[0.1,0.1,0.1,0.0025],ki=[0.000,0.000,0.000,0.000]):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.drone = drone()
        self.breaker = True
        self.vel = [0.0, 0.0, 0.0, 0.0]
        self.prev_time = [0.0, 0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0, 0.0]
        self.e_i = [0.0, 0.0, 0.0, 0.0]
        self.speed = 5
        self.repeat_pilot = False
        self.single_point_error = 0.01
        self.multiple_point_error = 0.1 #0.1
        self.ang_error = 0.02 #0.02
        self.length = 1
        
    def autopilot(self,autopilot_data):
        self.breaker = autopilot_data.autopilot_breaker_state
        self.count_loop = 0
        self.reach_pose = False

        listener = tf.TransformListener()
        arrA = []
        arrB = []
        t = time.time()
        rate = rospy.Rate(50)
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = autopilot_data.newpos[0]
        target_pose.pose.position.y = autopilot_data.newpos[1]
        target_pose.pose.position.z = autopilot_data.newpos[2]
        target_orient = autopilot_data.neworientation[:3]
        if autopilot_data.no_of_points > 1:
            self.repeat_pilot = True
            self.error_tol = self.multiple_point_error
            self.new_autopilot_newpos = autopilot_data.newpos[3:]
            self.new_autopilot_neworient = autopilot_data.neworientation[3:]
            self.points_left = len(self.new_autopilot_newpos)/3
            if autopilot_data.no_of_points > self.length:
                self.list_of_points = autopilot_data.newpos
                self.list_of_orients = autopilot_data.neworientation
                self.length = autopilot_data.no_of_points
            elif autopilot_data.no_of_points == 1:
                self.error_tol = self.single_point_error
                if self.repeat_pilot:
                    self.new_autopilot_newpos = self.list_of_points
                    self.new_autopilot_neworient = self.list_of_orients
                    self.points_left = len(self.list_of_points)/3
                    self.error_tol = self.multiple_point_error
            else:
                print("Invalid no. of points given!")
            print("point:",self.length - autopilot_data.no_of_points+1,"/",self.length)
            print((target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z), target_orient[2]*180/math.pi)

            while not self.breaker:
                try:
                    rospy.Subscriber('/autopilot_stop_cmd',pilot,self.callback)
                    self.current_global_pos = trans
                    target_pose_n = target_pose
                    target_point = pose_in_local_frame.pose.position
                    if autopilot_data.autopilot_mode == 'move+turn':
                        self.move_n_turn(target_point,target_orient,rot)
                    elif autopilot_data.autopilot_mode == 'no_turn':
                        self.no_turn(target_point)
                    elif autopilot_data.autopilot_mode == 'reach&turn':
                        self.reach_turn(target_point,target_orient,rot)
                    elif autopilot_data.autopilot_mode == 'turn&move':
                        self.turn_move(target_point,target_orient,rot)  
                    else:
                        print("Invalid or No Autopilot Mode Specified!")
                    t1 = time.time()-t
                    p = target_point.x
                    arrA.append(t1)
                    arrB.append(p)
                    if self.reach_pose:
                        break
                except (tf.LookupException, tf.ConnectivityException):
                    continue
                rate.sleep()
            if self.repeat_pilot == True and self.reach_pose == True and self.breaker == False:
                self.nextpoint(self.new_autopilot_newpos,self,new_autopilot_neworient,self.points_left,autopilot_data.autopilot_mode)
            else:
                self.breaker = True
                self.repeat_pilot = False
                self.reach_pose = False
                print("Nav done!")
                with open('expt_x_sim.csv','w') as f:
                    writer = csv.writer(f, delimiter='\t')
                    writer.writerows(zip(arrA,arrB))

    def callback(self,value):
        self.breaker = value.autopilot_breaker_state

    def calculate_error(self,error,i):
        current_time = time.time()
        dt = 0.0
        if self.prev_time[i] != 0.0:
            dt = current_time - self.prev_time[i]
        de = error - self.prev_error[i]

        e_p = self.kp[i] * error
        self.e_i[i] += error * dt
        e_d = 0
        if dt > 0:
            e_d = de/dt
        self.prev_time[i] = current_time
        self.prev_error[i] = error
        return e_p + (self.ki[i]*self.e_i[i]) + (self.kd[i]*e_d)

    def no_turn(self,target_pos):
        if not self.breaker:
            errors = (target_pos.x, target_pos.y, target_pos.z)
            for i in range(len(errors)):
                self.vel[i] = self.calculate_error(errors[i], i)
            if max(errors) > self.error_tol or min(errors) < -self.error_tol:
                self.drone.moveLinear(self.speed*self.vel[0], self.speed*self.vel[1], self.speed*self.vel[2])
            else:
                self.reach_pose = True
                print("Reached destination coords")

    def reach_turn(self,target_pos,target_orient,current_orient):
        if not self.breaker:
            yaw = math.atan2(2 * (current_orient[0] * current_orient[1] + current_orient[3] * current_orient[2]), current_orient[3] * current_orient[3] + current_orient[0] * current_orient[0] - current_orient[1] * current_orient[1] - current_orient[2] * current_orient[2])
            errors = (target_pos.x, target_pos.y, target_pos.z, target_orient[2]-yaw)
            for i in range(len(errors)):
                self.vel[i] - self.calculate_error(errors[i], i)
            if max(errors[:3]) > self.error_tol or min(errors[:3]) < -self.error_tol:
                self.drone.moveLinear(self.speed*self.vel[0], self.speed*self.vel[1], self.speed*self.vel[2])
            else:
                if abs(errors[3]) > self.ang_error:
                    self.drone.moveAngular(0,0,self.speed*self.vel[3])
                    print("Reached position, adjusting orientation...")
                else:
                    self.reach_pose = True
                    print("Reached destination pose")
    
    def turn_move(self,target_pos,target_orient,current_orient):
        if not self.breaker:
            yaw = math.atan2(2 * (current_orient[0] * current_orient[1] + current_orient[3] * current_orient[2]), current_orient[3] * current_orient[3] + current_orient[0] * current_orient[0] - current_orient[1] * current_orient[1] - current_orient[2] * current_orient[2])
            errors = (target_pos.x, target_pos.y, target_pos.z, target_orient[2]-yaw)
            for i in range(len(errors)):
                self.vel[i] - self.calculate_error(errors[i], i)
            if abs(errors[3]) > self.ang_error:
                self.drone.moveAngular(0,0,self.speed*self.vel[3])
            else:
                if max(errors[:3]) > self.error_tol or min(errors[:3]) < -self.error_tol:
                    self.drone.moveLinear(self.speed*self.vel[0], self.speed*self.vel[1], self.speed*self.vel[2])
                    print("Corrected orientation, moving to destination...")
                else:
                    self.reach_pose = True
                    print("Reached destination pose")

    def move_n_turn(self,target_pos,target_orient,current_orient):
        if not self.breaker:
            yaw = math.atan2(2 * (current_orient[0] * current_orient[1] + current_orient[3] * current_orient[2]), current_orient[3] * current_orient[3] + current_orient[0] * current_orient[0] - current_orient[1] * current_orient[1] - current_orient[2] * current_orient[2])
            errors = (target_pos.x, target_pos.y, target_pos.z, target_orient[2]-yaw)
            for i in range(len(errors)):
                self.vel[i] - self.calculate_error(errors[i], i)
            if max(errors) > self.error_tol or min(errors) < -self.error_tol:
                if self.count_loop % 2 == 0 or abs(errors[3]) < self.ang_error:
                    self.drone.moveLinear(self.speed*self.vel[0], self.speed*self.vel[1], self.speed*self.vel[2])
                else:
                    self.drone.moveAngular(0,0,self.speed*self.vel[3])
                self.count_loop += 1
            else:
                self.reach_pose = True
                print("Reached destination pose")
                print("Errors:- Linear: %f; Angular: %f"%(np.linalg.norm((errors[0],errors[1],errors[2])),errors[3]))

    def nextpoint(self,newpos,neworient,points_left,mode):
        loop_data = pilot(False,newpos,neworient.points_left,mode)
        self.autopilot(loop_data)
        print("Going to next point")

def listener(pd):
    rospy.init_node("mover", anonymous=True)
    rospy.Subscriber("/autopilot_start_cmd", pilot, pd.autopilot)
    print(">>Started PID Controller for drone...\n\tSinglePose Error tol:",pd.single_point_error,"\n\tMultiPose Error tol:",pd.multiple_point_error,"\n\tAngular Error:",pd.ang_error)
    print("\tAutoPilot Speed:",pd.speed)
    print("PID terms:",pd.kp,pd.kd,pd.ki)
    rospy.spin()

if __name__ == '__main__':
    pd = PID()
    listener(pd)