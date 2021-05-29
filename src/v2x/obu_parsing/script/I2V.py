#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import socket
from obu_msgs.msg import OBU_fusion
from std_msgs.msg import Header
from perception_msgs.msg import Obstacle, ObstacleArray


def generate_minus_one():
    # initialize header
    I2Vinfo_msg = OBU_fusion()
    tmp_header = Header()
    tmp_header.stamp = rospy.Time.now()
    tmp_header.frame_id = "fake"
    I2Vinfo_msg.header = tmp_header
    # initialize I2Vinfo_msg
    I2Vinfo_msg.light_state = -1
    I2Vinfo_msg.light_remain_time = -1
    I2Vinfo_msg.light_jd = -1
    I2Vinfo_msg.light_wd = -1
    I2Vinfo_msg.light_x = -1
    I2Vinfo_msg.light_y = -1
    I2Vinfo_msg.park_jd = -1
    I2Vinfo_msg.park_wd = -1
    I2Vinfo_msg.park_x = -1
    I2Vinfo_msg.park_y = -1
    I2Vinfo_msg.obu_jd = -1
    I2Vinfo_msg.obu_wd = -1
    I2Vinfo_msg.obu_angle = -1
    I2Vinfo_msg.obu_x = -1
    I2Vinfo_msg.obu_y = -1
    I2Vinfo_msg.obu_angle_rad = -1
    I2Vinfo_msg.event_jd = -1
    I2Vinfo_msg.event_wd = -1
    I2Vinfo_msg.event_x = -1
    I2Vinfo_msg.event_y = -1
    I2Vinfo_msg.event_length = -1
    I2Vinfo_msg.event_radius = -1
    I2Vinfo_msg.emergency_car_jd = -1
    I2Vinfo_msg.emergency_car_wd = -1
    I2Vinfo_msg.emergency_car_x = -1
    I2Vinfo_msg.emergency_car_y = -1
    I2Vinfo_msg.emergency_car_is_near = -1
    I2Vinfo_msg.terminal_jd = -1
    I2Vinfo_msg.terminal_wd = -1
    I2Vinfo_msg.terminal_x = -1
    I2Vinfo_msg.terminal_y = -1

    return I2Vinfo_msg


def InitializeSPAT(color, timing, light_jd, light_wd):
    I2Vinfo_msg = generate_minus_one()

    I2Vinfo_msg.light_state = color
    I2Vinfo_msg.light_remain_time = timing
    I2Vinfo_msg.light_jd = light_jd
    I2Vinfo_msg.light_wd = light_wd

    I2V_info_pub.publish(I2Vinfo_msg)
    rospy.loginfo("Publsh I2V_info message[%d, %d, %f, %f]", I2Vinfo_msg.light_state,
                  I2Vinfo_msg.light_remain_time, I2Vinfo_msg.light_jd, I2Vinfo_msg.light_wd)


def InitializeEvent(EventList):
    I2Vinfo_msg = generate_minus_one()

    if (EventList[0] is not None):
        print(EventList[0][0])
        #print(EventList[0][0].dtype)
        I2Vinfo_msg.park_jd = float(EventList[0][0])
        I2Vinfo_msg.park_wd = float(EventList[0][1])

    if (EventList[1] is not None):
        I2Vinfo_msg.obu_jd = EventList[1][0]
        I2Vinfo_msg.obu_wd = EventList[1][1]
        I2Vinfo_msg.obu_angle = EventList[1][2]

    if (EventList[2] is not None):
        I2Vinfo_msg.event_jd = EventList[2]

    if (EventList[3] is not None):
        I2Vinfo_msg.event_wd = EventList[3]

    if (EventList[4] is not None):
        I2Vinfo_msg.event_length = EventList[4]

    if (EventList[5] is not None):
        I2Vinfo_msg.event_radius = EventList[5]

    if (EventList[6] is not None):
        I2Vinfo_msg.emergency_car_jd = EventList[6]

    if (EventList[7] is not None):
        I2Vinfo_msg.emergency_car_wd = EventList[7]

    if (EventList[8] is not None):
        I2Vinfo_msg.terminal_jd = EventList[8]
    
    if (EventList[9] is not None):
        I2Vinfo_msg.terminal_wd = EventList[9]

    I2V_info_pub.publish(I2Vinfo_msg)
    rospy.loginfo(
        "Publsh I2V_info message[%f, %f]", I2Vinfo_msg.obu_jd, I2Vinfo_msg.obu_wd)


def recv_msg(udp_socket):
    recv_msg = udp_socket.recvfrom(1024)
    # 解码
    recv_ip = recv_msg[1]
    recv_msg = recv_msg[0].decode("utf-8")
    recv_msg = json.loads(recv_msg)
    # 显示接收到的数据
    # print(">>>%s:%s" % (str(recv_ip), recv_msg))
    # print("   ")
    # print(type(recv_msg))
    return recv_msg


def SPATprocess(Info):
    intersectionLon = Info['data']["lon"]
    intersectionLat = Info['data']["lat"]
    # left turn state
    left_Phase = Info['data']['phase'][0]['color']
    # straight state
    straight_Phase = Info['data']['phase'][1]['color']
    SignalTiming = Info['data']['phase'][0]['time']
    # print(intersectionPhase)
    # print(type(intersectionPhase))
    return left_Phase, SignalTiming, intersectionLon, intersectionLat


def EventProcess(Info):
    # get OBU GPS Info
    passanger_position = [-1,-1]
    EventLon = -1
    EventLat = -1
    EventRadius = -1
    EventLength = -1
    emer_car_jd = -1
    emer_car_wd = -1
    terminal_jd = -1
    terminal_wd = -1

    OBUGPS = []
    # OBUGPS[0] = GPS LON
    OBUGPS.append(Info['data']['lon'])
    # OBUGPS[1] = GPS LAT
    OBUGPS.append(Info['data']['lat'])
    # OBUGPS[2] = gps heading
    OBUGPS.append(Info['data']['hea'])
    # identify accident
    EventList = []
    flag = Info['data']['type']
    # print(flag)
    if flag == 10100003:
        emer_car_jd, emer_car_wd = emergencyVehicle(Info)
    elif flag == 10200405:
        passanger_position = passanger(Info)
    elif flag == 10200104:
        EventLon, EventLat, EventLength, EventRadius = frontEvent(Info)
    elif flag == 10300095:
        turningPoint(Info)
    elif flag == 10300096:
        terminal_jd,terminal_wd=terminal(Info)
    else:
        print('Type number error')

    # Eventlist[0]:parkInfoList, Eventlist[1]:obugpsInfo
    # Eventlist[2]:frontevent_jd, [3]:frontevent_wd
    #          [4]:frontevent_length, [5]:radius
    # Eventlist[6]:emer_car_jd, [7]:emer_car_wd
    # Eventlist[8]:terminal_jd, [9]:terminal_wd
    # Eventlist[10]:event type
    if(passanger_position is not None):
        EventList.append(passanger_position)
        #print(EventList)
    EventList.append(OBUGPS)
    if(EventLon is not None):
        EventList.append(EventLon)
    if(EventLat is not None):
        EventList.append(EventLat)
    if(EventLength is not None):
        EventList.append(EventLength)
    if(EventRadius is not None):
        EventList.append(EventRadius)
    if(emer_car_jd is not None):
        EventList.append(emer_car_jd)
    if(emer_car_wd is not None):
        EventList.append(emer_car_wd)
    if(terminal_jd is not None):
        EventList.append(terminal_jd)
    if(terminal_wd is not None):
        EventList.append(terminal_wd)

    # 标志 中文
    if flag == 10100003:
        EventList.append("emergency vehical")
    elif flag == 10200405:
        EventList.append('passanger call')
    elif flag == 10200104:
        EventList.append('accident font')
    elif flag == 10300095:
        EventList.append('middle way')
    elif flag == 10300096:
        EventList.append('terminal')

    return EventList


def emergencyVehicle(Info):
    emerVehLon = Info['data']['participant'][0]['lon']
    emerVehLat = Info['data']['participant'][0]['lat']
    emerVehSpd = Info['data']['participant'][0]['spd']
    emerVehHea = Info['data']['participant'][0]['hea']
    # print(emerVehHea, emerVehLat, emerVehLon)
    # print(Info['data']['voice'])
    return emerVehLon, emerVehLat


def passanger(Info):
    passanger_position = []
    passanger_position.append(Info['data']['participant'][0]['lon'])
    passanger_position.append(Info['data']['participant'][0]['lat'])
    return passanger_position


def frontEvent(Info):
    EventLon = Info['data']['participant'][0]['lon']
    EventLat = Info['data']['participant'][0]['lat']
    EventLength = Info['data']['reserved1']
    EventRadius = Info['data']['reserved2']
    return EventLon, EventLat, EventLength, EventRadius


def turningPoint(Info):
    TpLon = Info['data']['participant'][0]['lon']
    TpLat = Info['data']['participant'][0]['lat']
    return TpLon, TpLat


def terminal(Info):
    TerminalLon = Info['data']['participant'][0]['lon']
    TerminalLat = Info['data']['participant'][0]['lat']
    print("terminal")
    return TerminalLon, TerminalLat


def main():
    # 1. 创建套接字
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 2. 绑定本地信息
    udp_socket.bind(('192.168.3.100', 9090))
    while True:
        Info = recv_msg(udp_socket)

        
        if Info['tag'] == 2104:
            print(" get one SPAT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            color, timing, light_jd, light_wd = SPATprocess(Info)
            InitializeSPAT(color, timing, light_jd, light_wd)
        if Info['tag'] == 2105:
            right_type = [10100003, 10200405, 10200104, 10300095, 10300096]
            if(Info['data']['type'] in right_type):
                print(" get one event!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                EventList = EventProcess(Info)
                print(EventList)
                InitializeEvent(EventList)




if __name__ == "__main__":
    try:
        # initialize node
        # node name: I2V_publisher
        rospy.init_node('I2V_publisher', anonymous=True)
        # create publisher
        I2V_info_pub = rospy.Publisher('/I2V_info', OBU_fusion, queue_size=10)
        # main loop
        main()
    except rospy.ROSInterruptException:
        pass
