#!/usr/bin/env python
import rospy
import time

import Leap
from leap_motion.msg import leap
from leap_motion.msg import leapros

import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from sensor_msgs.msg import Image as Immsg
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

map_param_edge1 = np.array([0,0])
map_param_edge2 = np.array([5,0])


class ImageCreater:
    def __init__(self):
        self.map_x = 0
        self.map_y = 0
        self.hand_x = 0
        self.hand_y = 0
        self.send_pos_flag = False
        self.is_hand_touching = False
        self.is_robot_responsed = False
        self.size_of_img1 = 500
        self.size_of_img2 = 250
        self.robot_x = 0
        self.robot_y = 0
        self.joy_first_x = 0
        self.joy_first_z = 0
        self.joy_cur_x = 0
        self.joy_cur_z = 0
        self.joy_max_vel = 0.3
        self.joy_max_angv = 0.4
        self.is_touching = False
        self.is_target_set = False
        self.is_grabbing = False
        self.target_x = 1000
        self.target_y = 1000
        self.img_msg = Immsg()
        self.bridge = CvBridge()
        self.send_img = cv2.imread("/home/tashiro-y/researches/programs/workspace/catkin_ws/src/leap_robot_controller/scripts/field.jpg")
        self.send_img = cv2.resize(self.send_img, (self.size_of_img2, self.size_of_img2))
        # self.cv_img_buff = self.send_img
        self.cv_img_buff = cv2.imread("/home/tashiro-y/researches/programs/workspace/catkin_ws/src/leap_robot_controller/scripts/field.jpg")
        self.cv_img_buff = cv2.resize(self.cv_img_buff, (self.size_of_img2, self.size_of_img2))
        # self.is_hand_added = False
        # self.is_target_added = False
        self.image_pub = rospy.Publisher("/rviz_camera_stream/CameraPub", Immsg, queue_size = 1)
        self.pos_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
        self.joy_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    def set_robot_pos(self, x, y):
        self.robot_x = x
        self.robot_y = y

    def set_hand_pos(self, x, y):
        self.hand_x = x
        self.hand_y = y

    def conv_to_map(self):
        conv_ratio = 0.02
        self.map_x = -(self.target_x - 125) * conv_ratio
        self.map_y = -(self.target_y + 125) * conv_ratio
        print("(hand_x, hand_y) = " + str(self.target_x) + ", " + str(self.target_y) + ")")
        print("(map_x, map_y) = " + str(self.map_x) + ", " + str(self.map_y) + ")")

    def show_robot(self):
        x = (-1) * (self.robot_x/5*250 - 250)
        y = self.robot_y / 5 * 250 + 250
        x = int(x)
        y = int(y)
        cv2.rectangle(self.cv_img_buff, (x-5, y-5), (x+5, y+5), (0,255,100), -1)

    def send_pos(self):
        if self.send_pos_flag and self.is_target_set and not self.is_grabbing:
            self.conv_to_map()
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = self.map_x
            pose.pose.position.y = self.map_y
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            self.pos_pub.publish(pose)
            self.send_pos_flag = False

    def hand_pos_visualizer(self):
        # print("point mode")
        start = time.time()
        self.cv_img_buff = cv2.imread("/home/tashiro-y/researches/programs/workspace/catkin_ws/src/leap_robot_controller/scripts/field.jpg")
        self.cv_img_buff = cv2.resize(self.cv_img_buff, (self.size_of_img2, self.size_of_img2))
        color = (0,0,100)
        # cv2.circle(self.cv_img_buff, (int(0 + self.size_of_img2/2), int(0 + self.size_of_img2/2)), self.size_of_img2/50, (0,100,0),  -1)

        # Show blue circle when touching and save it as target pos
        if self.is_target_set:
            cv2.circle(self.cv_img_buff, (int(self.target_x + self.size_of_img2/2), int(-self.target_y + self.size_of_img2/2)), self.size_of_img2/50, (100,0,0),  -1)

        # show red circle when hovering
        elif self.is_touching:
            cv2.circle(self.cv_img_buff, (int(self.target_x + self.size_of_img2/2), int(-self.target_y + self.size_of_img2/2)), self.size_of_img2/50, (0,0,100), -1)

        # show robot position 
        if self.is_robot_responsed:
            self.show_robot()
        
        # convert and Publish
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_img_buff, "bgr8"))
        except CvBridgeError as e:
            print(e)

        Time = time.time() - start
        # print("elapsed_time:{0}".format(Time) + "[sec]")
        
    def joy_mode(self):
        input_forward = self.joy_cur_z - self.joy_first_z
        input_turn = self.joy_cur_x - self.joy_first_x
        input_forward = self.input_control(input_forward) * self.joy_max_vel
        input_turn = self.input_control(input_turn) *self.joy_max_angv
        cmd_vel = Twist()
        cmd_vel.linear.x = input_forward
        cmd_vel.angular.z = input_turn
        self.joy_pub.publish(cmd_vel)
        # print("joy_mode")
        # print("forward = " + str(input_forward) + " turn = " + str(input_turn))

    def input_control(self, joy_input):
        limit = 60
        start_point = 10
        if abs(joy_input) < start_point:
            ret = 0
        elif abs(joy_input) > limit:
            if joy_input > 0:
                ret = -1.0
            elif joy_input < 0:
                ret = 1.0
        elif joy_input > 0:
            ret = (joy_input - start_point) / (limit - start_point)
        elif joy_input < 0:
            ret = (joy_input + start_point) / (limit - start_point) * (-1)
        return ret

    def show_image(self):
        # cv2.imshow('Leap Robot COntroller', self.map)
        # k = cv2.waitKey(0)
        a = 0


def LeapCallback(data):
    # Set index finger pos and convert to the image coordinate
    imc.hand_x = data.index_tip.x/100 * imc.size_of_img2/2
    imc.hand_y = (data.index_tip.y - 200)/100 * imc.size_of_img2/2
    raw_x = data.index_tip.x
    raw_y = data.index_tip.y
    # For out of image
    if imc.hand_x >= imc.size_of_img2/2 or imc.hand_x <= -imc.size_of_img2/2:
        if imc.hand_x > 0:
            imc.hand_x = imc.size_of_img2/2
        elif imc.hand_x < 0:
            imc.hand_x = -imc.size_of_img2/2
    if imc.hand_y >= imc.size_of_img2/2 or imc.hand_y <= -imc.size_of_img2/2:
        if imc.hand_y > 0:
            imc.hand_y = imc.size_of_img2/2
        elif imc.hand_y < 0:
            imc.hand_y = -imc.size_of_img2/2
    # print("(x , y) = ( " + str(imc.hand_x) +" " + str(imc.hand_y) + ")")  
    # print("(raw_x , raw_y) = ( " + str(raw_x) +" " + str(raw_y) + ")")  

    # print("(hand_x, hand_y) = " + str(imc.hand_x) + ", " + str(imc.hand_y) + ")")

    # Judge touching
    if data.index_tip.z < -30:
        # print("touching")
        imc.is_touching = True
        imc.is_target_set = False
        imc.target_x = imc.hand_x
        imc.target_y = imc.hand_y
        imc.send_pos_flag = False

    elif data.index_tip.z > -10:
        # print("Hovering")
        if imc.is_touching:
            imc.is_target_set = True
        imc.is_touching = False
    # print(data.index_tip.z)

    if data.circle_state and data.index_tip.z  > 20:
        print("set the send_pos_flag")
        imc.send_pos_flag = True

    if data.sphere_radius >0 and data.sphere_radius < 38:
        if not imc.is_grabbing:
            imc.joy_first_x = data.palmpos.x
            imc.joy_first_z = data.palmpos.z
        imc.is_grabbing = True
        imc.joy_cur_x = data.palmpos.x
        imc.joy_cur_z = data.palmpos.z
    elif data.sphere_radius > 20:
        imc.is_grabbing = False
    # print(data.sphere_radius)

def RoboPosCallback(data):
    print("x = " + str(data.pose.pose.position.x) + " y = " + str(data.pose.pose.position.y))
    imc.robot_x = data.pose.pose.position.x
    imc.robot_y = data.pose.pose.position.y
    imc.is_robot_responsed = True


def Interface():
    rospy.init_node('leap_robot_controller', anonymous=True)
    r = rospy.Rate(30)
    rospy.Subscriber("leapmotion/data", leapros, LeapCallback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, RoboPosCallback)

    while not rospy.is_shutdown():
        if imc.is_grabbing:
            imc.joy_mode()
        elif not imc.is_grabbing:
            imc.hand_pos_visualizer()
            imc.send_pos()
        r.sleep()

imc = ImageCreater()

if __name__ == '__main__':
    try:
        Interface()
    except rospy.ROSInterruptException: pass

