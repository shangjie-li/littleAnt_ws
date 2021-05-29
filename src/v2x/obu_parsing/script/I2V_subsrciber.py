#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import socket
from  obu_msgs.msg import obu

def I2VInfoCallback(I2Vinfo_msg):
    rospy.loginfo("Subcribe obu Info:color:%s  timing:%d  passanger_num:%d", 
			I2Vinfo_msg.light_state, I2Vinfo_msg.light_remain_time, I2Vinfo_msg.park_id)

def person_subscriber():
	# ROS节点初始化
    rospy.init_node('I2V_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/I2V_info", obu, I2VInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()
