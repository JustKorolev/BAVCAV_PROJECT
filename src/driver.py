#!/usr/bin/env python3

import time
from djitellopy import tello
import rospy
import cv2
import glob
from cv_bridge import CvBridge
from Final_Project.msg import Flip 
from Final_Project.msg import State
from Final_Project.msg import Control
from Final_Project.msg import Mode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import os
import random


class Driver():


    def __init__(self):
        self.drone = tello.Tello()
        self.drone.connect(False)
        self.drone.streamon()
        rospy.init_node("driver", anonymous=True) #inits ros node

        self.bridge = CvBridge()

        self.state_pub = rospy.Publisher("tello/state", State, queue_size=10)
        self.front_feed_pub = rospy.Publisher("tello/camera", Image, queue_size=5)

        rospy.Subscriber("tello/vel", Twist, self.vel_callback)
        rospy.Subscriber("tello/dist", Twist, self.dist_callback)
        rospy.Subscriber("tello/land", Empty, self.land_callback)
        rospy.Subscriber("tello/takeoff", Empty, self.takeoff_callback)
        rospy.Subscriber("tello/flip", Flip, self.flip_callback)
        rospy.Subscriber("tello/control", Control, self.control_callback)
        rospy.Subscriber("tello/mode", Mode, self.mode_callback)

        self.state_msg = State()

        self.drone.TIME_BTW_RC_CONTROL_COMMANDS = 0.1
        self.cap_num = random.randint(0,1000)
        self.temp_num = 0
        self.vid_num = 0
        self.recording = False
        self.prev_vel = None

        self.current_path = "/home/akorolev/BWSI_Student_Code/catkin_ws/src/Final_Project"


    def set_state(self):
        self.state_msg.battery = self.drone.get_battery()
        self.state_msg.height = self.drone.get_height()
        
        self.state_pub.publish(self.state_msg)


    def set_feed(self):
        if(self.drone.stream_on):
            feed = self.drone.get_frame_read().frame
            cv2.imshow("feed", feed)
            cv2.waitKey(1)
            path = os.path.join(self.current_path,"temp/tello_photo_{}.png".format(self.temp_num))
            if(self.recording):
                cv2.imwrite(path,feed)
                self.temp_num += 1
            front_feed_msg = self.bridge.cv2_to_imgmsg(feed, "bgr8")
            self.front_feed_pub.publish(front_feed_msg)
    

    def vel_callback(self, vel):
        if(self.drone.is_flying == True):
            if (vel != self.prev_vel):
                self.drone.send_rc_control(int(vel.linear.y),int(vel.linear.x),int(vel.linear.z),int(vel.angular.z))
        self.prev_vel = vel


    def dist_callback(self, data):
        if(self.drone.is_flying == True):
            if(data.linear.x >= 20):
                self.drone.move_forward(int(data.linear.x))
            elif(data.linear.x <= -20):
                self.drone.move_back(-int(data.linear.x))
            if(data.linear.y >= 20):
                self.drone.move_right(int(data.linear.y))
            elif(data.linear.y <= -20):
                self.drone.move_left(-int(data.linear.y))
            if(data.linear.z >= 20):
                self.drone.move_up(int(data.linear.z))
            elif(data.linear.z <= -20):
                self.drone.move_down(-int(data.linear.z))

            if(data.angular.z > 0):
                self.drone.rotate_clockwise(int(data.angular.z))
            elif(data.angular.z < 0):
                self.drone.rotate_counter_clockwise(-int(data.angular.z))



    def land_callback(self, data):
        if(self.drone.is_flying == True):
            self.drone.land()
            print("landed")
            self.drone.is_flying = False
 

    def takeoff_callback(self, data):
        self.drone.send_rc_control(0,0,0,0)
        self.drone.takeoff()
        self.drone.is_flying = True
        

    def flip_callback(self, flip):
        if(self.drone.is_flying == True):
            if(flip.direction == "f"):
                self.drone.flip_forward()
            elif(flip.direction == "l"):
                self.drone.flip_left()
            elif(flip.direction == "r"):
                self.drone.flip_right()
            elif(flip.direction == "b"):
                self.drone.flip_back()


    def mode_callback(self,data):
        if(data.command == "photo"):
            feed = self.drone.get_frame_read().frame
            path = os.path.join(self.current_path,"photos/tello_photo_{}.png".format(self.cap_num))
            print(self.current_path)
            cv2.imwrite(path,feed)
            self.cap_num = random.randint(0,1000)
        if(data.command == "video_start"):
            self.recording = True
        if(data.command == "video_stop"):
            self.recording = False
            img_array = []
            path = os.path.join(self.current_path,"temp/*.png")
            for filename in glob.glob(path):
                img = cv2.imread(filename)
                height, width, layers = img.shape
                size = (width,height)
                img_array.append(img)

            path = os.path.join(self.current_path,"videos/tello_video_{}.png".format(self.vid_num))
            out = cv2.VideoWriter(f'./videos/tello_vid_{self.vid_num}',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)
            for i in range(len(img_array)):
                out.write(img_array[i])
            self.vid_num += 1


    def control_callback(self, control):
        if(self.drone.is_flying == True):
            if(control.control == "emergency" and self.drone.get_height() > 150):
                self.drone.send_command_without_return("emergency")
                self.drone.land()
                self.drone.takeoff()


if __name__ == "__main__":
    driver = Driver()
    driver.drone.send_rc_control(0,0,0,0)
    while(not rospy.is_shutdown()):
        try:
            driver.set_state()
            driver.set_feed()
        except rospy.ROSInterruptException:
            break
    #driver.drone.streamoff()
    cv2.destroyAllWindows()




