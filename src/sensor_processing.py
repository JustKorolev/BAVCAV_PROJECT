#!/usr/bin/env python3

import time
from cv2 import FlannBasedMatcher, inRange
from djitellopy import tello
import rospy
import cv2
from cv_bridge import CvBridge
from Final_Project.msg import Flip 
from Final_Project.msg import Mode
from Final_Project.msg import Control
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import mediapipe as mp
import speech_recognition as sr
import math
import copy
import os
from pocketsphinx import LiveSpeech, get_model_path


class Sensors():

    def __init__(self):

        rospy.init_node("sensor_processing", anonymous=True) #inits ros node
        self.bridge = CvBridge()

        rospy.Subscriber("tello/camera", Image, self.camera_callback)
        rospy.Subscriber("tello/control", Control, self.control_callback)

        self.mode_pub = rospy.Publisher("tello/mode", Mode, queue_size = 1)
        self.pose_pub = rospy.Publisher("tello/pose2D", Pose2D, queue_size = 5)
        self.land_pub = rospy.Publisher("tello/land", Empty, queue_size = 5)
        self.takeoff_pub = rospy.Publisher("tello/takeoff", Empty, queue_size = 5)

        self.mode_msg = Mode()
        self.pose_msg = Pose2D()

        self.FOV = (82.6 * (math.pi/180))
        self.delta_x = 0
        self.delta_y = 0
        self.time_0 = rospy.Time.now().to_sec()
        self.command_delay = 1.5
        
        self.control = ""
        self.hand_array = None

        #pocketsphin init
        self.r = sr.Recognizer()
        model_path = get_model_path()
        self.speech = LiveSpeech(
            verbose=False,
            sampling_rate=16000,
            buffer_size=2048,
            no_search=False,
            full_utt=False,
            hmm=os.path.join(model_path, 'en-us'),

            #make sure to change these to your path to the words files
            lm=os.path.join("/home/akorolev/BWSI_Student_Code/catkin_ws/src/Final_Project/speech/words.lm"),
            dic=os.path.join("/home/akorolev/BWSI_Student_Code/catkin_ws/src/Final_Project/speech/words.dic")
        )
            #change path for your files^^^

        #mediapipe init
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        
        #finger palm-boundary booleans
        self.t = False
        self.i = False
        self.m = False
        self.r = False
        self.p = False

        #rospy.spin()

    
    def set_hand_array(self,image):
        with self.mp_hands.Hands(model_complexity=0,min_detection_confidence=0.5,min_tracking_confidence=0.5) as hands:

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image)

            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Iterate through hands to format coordinate data
            hand_array = []
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    point_array = []
                    for point in hand_landmarks.landmark:
                        point_array.append((point.x, point.y))
                    hand_array.append(point_array)
                        
        return hand_array



    def set_voice(self):
        mode = ""
        command = ""

        for phrase in self.speech:
            phrase = str(phrase)
            print(phrase)

            if 'COMMAND' in phrase:

                #the simple movement commands
                if 'COMMAND START' in phrase:
                    mode = 'simple_movement'
                    command = 'takeoff'
                if 'COMMAND LANDING' in phrase:
                    mode = 'simple_movement'
                    command = 'landing'
                if 'COMMAND FORWARD' in phrase:
                    mode = 'simple_movement'
                    command = 'forward'
                if 'COMMAND BACKWARD' in phrase:
                    mode = 'simple_movement'
                    command = 'backward'
                if 'COMMAND LEFT' in phrase:
                    mode = 'simple_movement'
                    command = 'left'
                if 'COMMAND RIGHT' in phrase:
                    mode = 'simple_movement'
                    command = 'right'
                if 'COMMAND UP' in phrase or 'COMMAND OFF' in phrase:
                    mode = 'simple_movement'
                    command = 'up'
                if 'COMMAND DOWN' in phrase:
                    mode = 'simple_movement'
                    command = 'down'
                if 'COMMAND TURN LEFT' in phrase:
                    mode = 'simple_movement'
                    command = 'turn left'
                if 'COMMAND TURN RIGHT' in phrase:
                    mode = 'simple_movement'
                    command = 'turn right'
                if 'COMMAND TURN NINETY LEFT' in phrase:
                    mode = 'simple_movement'
                    command = 'turn 90 degrees left'
                if 'COMMAND TURN NINETY RIGHT' in phrase:
                    mode = 'simple_movement'
                    command = 'turn 90 degrees right'
                if 'COMMAND TURN ONE EIGHTY LEFT' in phrase:
                    mode = 'simple_movement'
                    command = 'turn 180 degrees left'
                if 'COMMAND TURN ONE EIGHTY RIGHT' in phrase:
                    mode = 'simple_movement'
                    command = 'turn 180 degrees right'
                if 'COMMAND THREE SIXTY LEFT' in phrase:
                    mode = 'simple_movement'
                    command = 'turn 360 degrees left'
                if 'COMMAND TURN THREE SIXTY RIGHT' in phrase:
                    mode = 'simple_movement'
                    command = 'turn 360 degrees right'
                if 'COMMAND STOP' in phrase:
                    mode = 'simple_movement'
                    command = 'stop'

                #the trick commands
                if 'COMMAND FLIP FORWARD' in phrase:
                    mode = 'tricks'
                    command = 'flip forward'
                if 'COMMAND FLIP BACKWARD' in phrase:
                    mode = 'tricks'
                    command = 'flip backward'
                if 'COMMAND FLIP LEFT' in phrase:
                    mode = 'tricks'
                    command = 'flip left'
                if 'COMMAND FLIP RIGHT' in phrase:
                    mode = 'tricks'
                    command = 'flip right'
                if 'COMMAND FREEFALL' in phrase:
                    mode = 'tricks'
                    command = 'freefall'
                if 'COMMAND BOUNCE' in phrase:
                    mode = 'tricks'
                    command = 'bounce'
                # if 'COMMAND WAVE' in phrase:
                #     mode = 'tricks'
                #     command = 'wave'
                # if 'COMMAND SPIRAL' in phrase:
                #     mode = 'tricks'
                #     command = 'spiral'

                #the geometric movement commands
                if 'COMMAND CIRCLE' in phrase:
                    mode = 'geometric_movement'
                    command = 'circle'
                if 'COMMAND SQUARE' in phrase:
                    mode = 'geometric_movement'
                    command = 'square'
                if 'COMMAND TRIANGLE' in phrase:
                    mode = 'geometric_movement'
                    command = 'triangle'
                if 'COMMAND VERTICAL SQUARE' in phrase:
                    mode = 'geometric_movement'
                    command = 'vertical square'

                #the follow and target commands
                if 'COMMAND FOLLOW' in phrase:
                    mode = 'follow'
                    command = 'no_command'
                if 'COMMAND FIND TARGET' in phrase:
                    mode = 'find_target'
                    command = 'no_command'
                if 'COMMAND TARGET' in phrase:
                    mode = 'target'
                    command = 'no_command'

                #the photo and video commands
                if 'COMMAND PHOTO' in phrase:
                    mode = 'no_mode'
                    command = 'photo'
                if 'COMMAND VIDEO START' in phrase:
                    mode = 'no_mode'
                    command = 'video_start'
                if 'COMMAND VIDEO STOP' in phrase:
                    mode = 'no_mode'
                    command = 'video_stop'


            break #breaks so it doesn't loop through an infinite amount
                    
        #print(mode_command)    
        return mode,command
                    

    def set_mode(self,mode,command):
        self.time_1 = rospy.Time.now().to_sec()
        if(self.time_1 - self.time_0 < self.command_delay):
            self.mode_msg.mode = ""
            self.mode_msg.command = ""
        else:
            self.mode_msg.mode = mode
            self.mode_msg.command = command
            print(mode + "," + command)
            self.time_0 = rospy.Time.now().to_sec()
        self.mode_pub.publish(self.mode_msg)


    def set_pose(self,hand_array):
        try:
            x = hand_array[0][9][0]
            y = hand_array[0][9][1]
            self.pose_msg.x = 0
            self.pose_msg.y = y
            self.pose_msg.theta = x
            self.pose_pub.publish(self.pose_msg)
        except IndexError:
            pass


    def camera_callback(self,camera):
        self.feed = self.bridge.imgmsg_to_cv2(camera, desired_encoding='passthrough')
        self.hand_array = self.set_hand_array(self.feed)

        self.set_pose(self.hand_array)

    
    def control_callback(self,data):
        self.control = data.control


    def find_slope(self, hand_arr, point_list):
        arr = []
        try:
            for i in point_list:
                arr.append(hand_arr[0][i])
            self.delta_y = arr[1][1] - arr[0][1]
            self.delta_x = arr[1][0] - arr[0][0]
            try:
                slope = self.delta_y/self.delta_x
            except ZeroDivisionError:
                slope = 100
            #print(slope)
        except IndexError:
            slope = ""
        return slope


    def find_direction(self, hand_arr, point_list):
        slope = self.find_slope(hand_arr, point_list)
        if slope != "":
            if -1 < slope < 1:
                if self.delta_x < 0:
                    return "right"
                else:
                    return "left"
            else:
                if self.delta_y < 0:
                    return "up"
                else:
                    return "down"
        else:
            return ""
                    
    
    def finger_activation(self, hand_arr):
        tolarance_1 = 0.05
        arr = []
        try:
            #checks if hand is horizontal, flips x and y coordinates
            if -1 < self.find_slope(hand_arr, [0, 9]) < 1:
                palm_x = [hand_arr[0][0][0], hand_arr[0][9][0]]
                palm_y = [hand_arr[0][1][1], hand_arr[0][17][1]]
            else:
                palm_y = [hand_arr[0][0][1], hand_arr[0][9][1]]
                palm_x = [hand_arr[0][1][0], hand_arr[0][17][0]]
            list_y = sorted(palm_y)
            list_x = sorted(palm_x)
            list_y[0] = list_y[0] - tolarance_1
            list_y[1] = list_y[1] + tolarance_1
            list_x[0] = list_x[0] - tolarance_1
            list_x[1] = list_x[1] + tolarance_1

            #creates an array of only tips of each finger
            for i in [4, 8, 12, 16, 20]:
                arr.append(hand_arr[0][i])

            #check if fingers are in boundary boxes
            if ((list_y[0]) < arr[0][1] < (list_y[1])) and ((list_x[0]) < arr[0][0] < (list_x[1])):
                self.t = False
            else:
                self.t = True
            if (list_y[0] < arr[1][1] < list_y[1]) and (list_x[0] < arr[1][0] < list_x[1]):
                self.i = False
            else:
                self.i = True
            if (list_y[0] < arr[2][1] < list_y[1]) and (list_x[0] < arr[2][0] < list_x[1]):
                self.m = False
            else:
                self.m = True
            if (list_y[0] < arr[3][1] < list_y[1]) and (list_x[0] < arr[3][0] < list_x[1]):
                self.r = False
            else:
                self.r = True
            if ((list_y[0]) < arr[4][1] < (list_y[1])) and ((list_x[0]) < arr[4][0] < (list_x[1])):
                self.p = False
            else:
                self.p = True
        except TypeError:
            self.t = False
            self.i = False
            self.m = False
            self.r = False
            self.p = False
            

    def set_hand_gesture(self, hand_arr):
        # checks if the finger is activated
        # checks if other fingers are deactivated
        # checks for the direction of finger

        self.hand_gesture = ""
        mode = ""
        command = ""
        # thumb - simple_movement
        if (self.t) and (not self.i) and (not self.m) and (not self.r) and (not self.p):
            if (self.find_direction(hand_arr, [2, 4]) == "up"):
                command = "takeoff"
            elif (self.find_direction(hand_arr, [2, 4]) == "down"):
                command = "land"
            elif (self.find_direction(hand_arr, [2, 4]) == "right"):
                command = "turn left"
            elif (self.find_direction(hand_arr, [2, 4]) == "left"):
                command = "turn right"
            mode = "simple_movement"

        # thumb & index - simple_movement
        elif (self.t) and (self.i) and (not self.m) and (not self.r) and (not self.p):
            if (self.find_direction(hand_arr, [5, 8]) == "up"):
                command = "forward"
            elif (self.find_direction(hand_arr, [5, 8]) == "down"):
                command = "backward"
            elif (self.find_direction(hand_arr, [5, 8]) == "right"):
                command = "turn 90 degrees left"
            elif (self.find_direction(hand_arr, [5, 8]) == "left"):
                command = "turn 90 degrees right"
            mode = "simple_movement"


        # index - simple_movement
        elif (not self.t) and (self.i) and (not self.m) and (not self.r) and (not self.p):
            if (self.find_direction(hand_arr, [5, 8]) == "up"):
                command = "up"
            elif (self.find_direction(hand_arr, [5, 8]) == "down"):
                command = "down"
            elif (self.find_direction(hand_arr, [5, 8]) == "right"):
                command = "left"
            elif (self.find_direction(hand_arr, [5, 8]) == "left"):
                command = "right"
            mode = "simple_movement"

        # all - simple_movement
        if (self.t) and (self.i) and (self.m) and (self.r) and (self.p):
            mode = "simple_movement"
            command = "stop"

        # index & middle - tricks
        elif (self.i) and (self.m) and (not self.r) and (not self.p):
            if (self.find_direction(hand_arr, [5, 8]) == "up"):
                command = "bounce"
            elif (self.find_direction(hand_arr, [5, 8]) == "down"):
                command = "freefall"
            elif (self.find_direction(hand_arr, [5, 8]) == "right"):
                command = "wave"
            elif (self.find_direction(hand_arr, [5, 8]) == "left"):
                command = "spiral"
            mode = "tricks"

        # index & middle & ring - geometric_movement
        elif (self.i) and (self.m) and (self.r) and (not self.p):
            if (self.find_direction(hand_arr, [5, 8]) == "up"):
                command = "circle"
            elif (self.find_direction(hand_arr, [5, 8]) == "down"):
                command = "vertical square"
            elif (self.find_direction(hand_arr, [5, 8]) == "right"):
                command = "square"
            elif (self.find_direction(hand_arr, [5, 8]) == "left"):
                command = "triangle"
            mode = "geometric_movement"

        # pinky - tricks
        elif (not self.i) and (not self.m) and (not self.r) and (self.p):
            if (self.find_direction(hand_arr, [17, 20]) == "up"):
                command = "flip forward"
            elif (self.find_direction(hand_arr, [17, 20]) == "down"):
                command = "flip backward"
            elif (self.find_direction(hand_arr, [17, 20]) == "right"):
                command = "flip left"
            elif (self.find_direction(hand_arr, [17, 20]) == "left"):
                command = "flip right"
            mode = "tricks"

        print(mode + "," + command)
        return mode,command


if __name__ == "__main__":
    sensors = Sensors()
    while(not rospy.is_shutdown()):
        try:
            sensors.time_1 = rospy.Time.now().to_sec()
            if(sensors.time_1 - sensors.time_0 > sensors.command_delay):
                sensors.finger_activation(sensors.hand_array)
                mode,command = sensors.set_hand_gesture(sensors.hand_array)

                if(command != ""):
                    sensors.set_mode(mode,command)

                mode,command = sensors.set_voice()
                if(command != ""):
                    sensors.set_mode(mode,command)
            
        except rospy.ROSInterruptException:
            pass
