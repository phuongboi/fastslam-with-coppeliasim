import rospy
import math
import random
import numpy as np
from collections import deque
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from geometry_msgs.msg import Transform
class VrepEnvironment():
    def __init__(self, speed, turn, rate):
        #self.scan_sub = rospy.Subscriber('scanData', Float32MultiArray, self.scan_callback)
        self.left_pub = rospy.Publisher('leftMotorSpeed', Float32, queue_size=1, latch=True)
        self.right_pub = rospy.Publisher('rightMotorSpeed', Float32, queue_size=1, latch=True)
        self.fifo = []
        rospy.init_node('pioneer_controller')
        self.v_forward = speed
        self.v_turn = turn
        self.rate = rospy.Rate(rate) # frequence of motor speed publisher

    def step(self, action):

        if action==0:
            self.left_pub.publish(self.v_forward-self.v_turn)
            self.right_pub.publish(self.v_forward+self.v_turn)
            self.rate.sleep()
            # self.left_pub.publish(0)
            # self.right_pub.publish(0)
            #self.rate.sleep()
        elif action==1:
            self.left_pub.publish(self.v_forward)
            self.right_pub.publish(self.v_forward)
            self.rate.sleep()
            # self.left_pub.publish(0)
            # self.right_pub.publish(0)
            #self.rate.sleep()
        elif action==2:
            self.left_pub.publish(self.v_forward+self.v_turn)
            self.right_pub.publish(self.v_forward-self.v_turn)
            self.rate.sleep()
            # self.left_pub.publish(0)
            # self.right_pub.publish(0)
            #self.rate.sleep()

        elif action==3:
            self.left_pub.publish(-self.v_forward)
            self.right_pub.publish(-self.v_forward)
            self.rate.sleep()
            self.left_pub.publish(0)
            self.right_pub.publish(0)
            #self.rate.sleep()

        elif action==4:
            self.left_pub.publish(0)
            self.right_pub.publish(0)
            self.rate.sleep()
        elif action==5:
            print("do nothing")


        transform = rospy.wait_for_message('transformData', Transform)
        scan = rospy.wait_for_message('scanData',Float32MultiArray).data
        return transform, scan



    # def scan_callback(self, msg):
    #     # FIFO queue storing DVS data during one step
    #     self.fifo.append(msg.data)
    #     self.fifo.popleft()
    #     return
