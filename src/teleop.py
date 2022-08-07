#!/usr/bin/env python3

from this import d
import pygame
from djitellopy import tello
from re import L
import rospy
from Final_Project.msg import Flip 
from Final_Project.msg import State
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys
import time



pygame.init()
display = (640,360)
pygame.display.set_mode(display)

class Teleop:

    def __init__(self):

        rospy.init_node("teleop", anonymous=True) #inits ros node

        rospy.Subscriber("tello/state", State, self.state_callback)

        self.flip_pub = rospy.Publisher("tello/flip", Flip, queue_size = 5)
        self.vel_pub = rospy.Publisher("tello/vel", Twist, queue_size = 4)
        self.land_pub = rospy.Publisher("tello/land", Empty, queue_size = 5)
        self.takeoff_pub = rospy.Publisher("tello/takeoff", Empty, queue_size = 5)

        self.flip_msg = Flip()
        self.vel_msg = Twist()

        self.terminate = False
        self.velocity = 90


    def set_vel(self):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_a]:
            self.vel_msg.linear.y = -self.velocity
        elif keys[pygame.K_d]:
            self.vel_msg.linear.y = self.velocity
        if keys[pygame.K_w]:
            self.vel_msg.linear.x = self.velocity
        elif keys[pygame.K_s]:
            self.vel_msg.linear.x = -self.velocity
        if keys[pygame.K_UP]:
            self.vel_msg.linear.z = self.velocity
        elif keys[pygame.K_DOWN]:
            self.vel_msg.linear.z = -self.velocity

        if keys[pygame.K_LEFT]:
            self.vel_msg.angular.z = -self.velocity
        elif keys[pygame.K_RIGHT]:
            self.vel_msg.angular.z = self.velocity

        self.vel_pub.publish(self.vel_msg)

    
    def set_land(self):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_q]:
            self.terminate = True
            self.land_pub.publish()

    
    def set_takeoff(self):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LSHIFT]:
            self.takeoff_pub.publish()
            time.sleep(3)

    
    def set_flip(self):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_8]:
            self.flip_msg.direction = "f"
        elif keys[pygame.K_4]:
            self.flip_msg.direction = "l"
        elif keys[pygame.K_6]:
            self.flip_msg.direction = "r"
        elif keys[pygame.K_2]:
            self.flip_msg.direction = "b"

        if(self.flip_msg.direction != ""):
            self.flip_pub.publish(self.flip_msg)


    def set_defaults(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.z = 0
        self.flip_msg.direction = ""
        

    def state_callback(self, state):
        print(state)
    

if __name__ == "__main__":
    teleop = Teleop()
    t0 = rospy.Time.now().to_sec()
    teleop.set_defaults()
    while(not rospy.is_shutdown()):
        try:
            #teleop.set_vel()
            #teleop.set_flip()
            teleop.set_land()
            teleop.set_takeoff()
            #teleop.set_defaults()
            pygame.event.pump()
        except rospy.ROSInterruptException:
            pass
    pygame.quit()
