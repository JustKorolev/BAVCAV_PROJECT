#!/usr/bin/env python3

from this import d
import pygame
from djitellopy import tello
from re import L
import rospy
from Final_Project.msg import Flip 
from Final_Project.msg import State
from Final_Project.msg import Mode
from Final_Project.msg import Control
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import simple_pid
import sys
import time
import copy


class Movement:

    def __init__(self):

        rospy.init_node("movement", anonymous=True) #inits ros node

        rospy.Subscriber("tello/mode", Mode, self.mode_callback)
        rospy.Subscriber("tello/pose2D", Pose2D, self.pose2D_callback)

        self.flip_pub = rospy.Publisher("tello/flip", Flip, queue_size = 5)
        self.vel_pub = rospy.Publisher("tello/vel", Twist, queue_size = 5)
        self.dist_pub = rospy.Publisher("tello/dist", Twist, queue_size = 5)
        self.land_pub = rospy.Publisher("tello/land", Empty, queue_size = 5)
        self.takeoff_pub = rospy.Publisher("tello/takeoff", Empty, queue_size = 5)
        self.control_pub = rospy.Publisher("tello/control", Control, queue_size = 5)

        self.control_msg = Control()
        self.flip_msg = Flip()
        self.vel_msg = Twist()
        self.dist_msg = Twist()
        self.mode_msg = Mode()
        self.pose2D_msg = Pose2D()
        self.camera_msg = Image()

        self.velocity = 50
        self.prev_vel_msg = None
        self.prev_dist_msg = None
        self.mode = ""
        self.command = ""
        self.prev_mode = "mode"
        self.prev_command = "command"

        #PID initialization
        self.pid_angle = simple_pid.PID(50,.5)
        self.pid_height = simple_pid.PID(1.2)
        self.pid_angle.SetPoint = 0
        self.pid_height.SetPoint = 360
        self.pid_angle.setSampleTime(0.01)
        self.pid_height.setSampleTime(0.01)


    def set_defaults(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.z = 0
        self.flip_msg.direction = ""
        self.control_msg = ""

        self.dist_msg.linear.x = 0
        self.dist_msg.linear.y = 0
        self.dist_msg.linear.z = 0
        self.dist_msg.angular.z = 0

        self.target_detected = False
        self.delta_x = 0
        self.delta_y = 0
        self.delta_theta = 0

        self.mode = ""
        self.command = ""



    def set_land(self):
            self.land_pub.publish()

    
    def set_takeoff(self):
            self.takeoff_pub.publish()


    def set_dist(self,x,y,z,theta):
        self.dist_msg.linear.x = x
        self.dist_msg.linear.y = y
        self.dist_msg.linear.z = z
        self.dist_msg.angular.z = theta
        self.dist_pub.publish(self.dist_msg)


    def set_vel(self,x,y,z,theta):
        self.vel_msg.linear.x = x
        self.vel_msg.linear.y = y
        self.vel_msg.linear.z = z
        self.vel_msg.angular.z = theta
        if self.vel_msg != self.prev_vel_msg:
            self.vel_pub.publish(self.vel_msg)
            self.prev_vel_msg = copy.deepcopy(self.vel_msg)

    
    def set_pid_angle(self):
        self.pid_angle.update(self.delta_theta)
        print(self.pid_angle.output)
        self.set_vel(0,0,0,int(self.pid_angle.output))

    
    def set_flip(self,flip):
        self.flip_msg.direction = flip
        self.flip_pub.publish(self.flip_msg)


    def set_control(self,control):
        self.control_msg = control
        self.control_pub.publish(self.control_msg)
        self.control_msg = control


    def set_mode(self):
        if(self.command == self.prev_command):
            self.command = ""
        else:
            print("diff")
        self.prev_command = self.command


    def mode_callback(self, mode):
        self.mode = mode.mode
        self.command = mode.command
    

    def pose2D_callback(self, pose):
        self.delta_x = 0.5 - pose.x
        self.delta_y = 0.5 - pose.y
        self.delta_theta = 0.5 - pose.theta
        self.target_detected = True


    def find_target(self):
        if self.target_detected == False:    
            self.set_vel(10,0,0,40)
            self.time_0 = rospy.Time.now().to_sec()
            while(self.target_detected == False):
                self.time_1 = rospy.Time.now().to_sec()
                if(self.time_1 - self.time_0 > 10):
                    print("stopped")
                    self.mode = ""
                    break
        else:
            pass


    def follow(self):
        set_distance = 100
        self.find_target()
        self.set_pid_angle()
    

    def target(self):
        pass


    def tricks(self):
        self.set_control(self.mode)
        if self.command == 'flip forward':
            self.set_flip('f')
        elif self.command == 'flip backward':
            self.set_flip('b')
        elif self.command == 'flip left':
            self.set_flip('l')
        elif self.command == 'flip right':
            self.set_flip('r')

        if self.command == "bounce":
            self.set_dist(0,0,self.velocity,0)
            self.set_dist(0,0,-self.velocity,0)
            self.set_dist(0,0,self.velocity,0)
            self.set_dist(0,0,-self.velocity,0)

        if self.command == "freefall":
            self.set_control("emergency")
            self.set_control("")
            self.set_vel(0,0,50,0)

        if self.command == "wave":   #sinusoidal wave not hand wave
            self.set_dist(0,0,0,-90)
            self.set_dist(self.velocity,0,0,90)
            self.set_dist(self.velocity,0,0,-90)
            self.set_dist(self.velocity,0,0,90)
            self.set_dist(self.velocity,0,0,-90)
            self.set_dist(0,0,0,90)

        if self.command == "spiral":
            self.set_dist(30,0,10,90)
            self.set_dist(30,0,-10,90)

        self.set_defaults()


    def simple_movement(self):
        self.set_control(self.mode)
        if self.command == 'takeoff':
            self.takeoff_pub.publish()
        elif self.command == 'land':
            self.land_pub.publish()
        elif self.command == 'forward':
            self.set_dist(self.velocity,0,0,0)
        elif self.command == 'backward':
            self.set_dist(-self.velocity,0,0,0)
        elif self.command == 'left':
            self.set_dist(0,-self.velocity,0,0)
        elif self.command == 'right':
            self.set_dist(0,self.velocity,0,0)
        elif self.command == 'up':
            self.set_dist(0,0,self.velocity,0)
        elif self.command == 'down':
            self.set_dist(0,0,-self.velocity,0)
        elif self.command == 'turn left':
            self.set_dist(0,0,0,-self.velocity)
        elif self.command == 'turn right':
            self.set_dist(0,0,0,self.velocity)
        elif self.command == 'turn 90 degrees left':
            self.set_dist(0,0,0,-90)
        elif self.command == 'turn 90 degrees right':
            self.set_dist(0,0,0,90)
        elif self.command == 'turn 180 degrees left':
            self.set_dist(0,0,0,-180)
        elif self.command == 'turn 180 degrees right':
            self.set_dist(0,0,0,180)
        elif self.command == 'turn 360 degrees left':
            self.set_dist(0,0,0,-360)
        elif self.command == 'turn 360 degrees right':
            self.set_dist(0,0,0,360)
        elif self.command == 'takeoff':
            self.set_takeoff()
        elif self.command == 'landing':
            self.set_land()
        elif self.command == 'stop':
            self.set_vel(0,0,0,0)
        else:
            self.set_vel(0,0,0,0)
            self.set_defaults()
            


    def geometric_movement(self):
        self.set_control(self.mode)
        if self.command == 'circle':
            self.set_vel(25,0,0,72)
            time.sleep(5)
            self.set_defaults()

        elif self.command == 'square':
            self.set_dist(-60,0,0,0)
            self.set_dist(0,60,0,0)
            self.set_dist(60,0,0,0)
            self.set_dist(0,-60,0,0)
            self.set_defaults()

        elif self.command == 'vertical square':
            self.set_dist(0,0,60,0)
            self.set_dist(-60,0,0,0)
            self.set_dist(0,0,-60,0)
            self.set_dist(60,0,0,0)
            self.set_defaults()

        elif self.command == 'triangle':
            self.set_dist(30,0,0,0)
            self.set_dist(0,0,0,60)
            self.set_dist(30,0,0,0)
            self.set_dist(0,0,0,60)
            self.set_dist(30,0,0,0)
            self.set_dist(0,0,0,60)
            self.set_defaults()
        else:
            pass
        
        
if __name__ == "__main__":
    movement = Movement()
    t0 = rospy.Time.now().to_sec()
    movement.set_defaults()
    while(not rospy.is_shutdown()):
        try:
            #print(movement.mode + "," + movement.command)
            if movement.mode == "follow":
                print("following")
                movement.follow()
            elif movement.mode ==  "target":
                movement.target
            elif movement.mode ==  "find_target":
                movement.find_target()
            elif movement.mode ==  "tricks":
                movement.tricks()
            elif movement.mode ==  "simple_movement":
                movement.simple_movement()
            elif movement.mode ==  "geometric_movement":
                movement.geometric_movement()
            else:
                movement.set_vel(0,0,0,0)
            movement.set_mode()
        except rospy.ROSInterruptException:
            pass
    pygame.quit()
