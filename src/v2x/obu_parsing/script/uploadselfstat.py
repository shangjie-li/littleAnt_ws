#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import socket
import time
import rospy

# def updateInfo():
# 	if updateflag == 1:
# 		print("Update state information")
# 		selfstat["data"]["device_id"] ="testB12138"
	
	# return selfstat

def main():
	#initialize node
	rospy.init_node('uploadselfstat', anonymous=True)
	#get hostname and ip automatically
	hostname = socket.gethostname()
	hostip = socket.gethostbyname(hostname)
	#print(hostip)
	#Create socket in client
	udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	udp_socket.bind((hostip, client_port))# bind client port
	#main loop for heartbeat upload
	while not rospy.is_shutdown():
		send_msg(udp_socket)
		time.sleep(1)
	

def send_msg(udp_socket):
	udp_socket.sendto(selfstat_json.encode("utf-8"),(dest_ip,dest_port))


if __name__ == '__main__':
	true = True
	false = False

	#initialize selfstate basic info 
	selfstat = 	{
			"data":{"absActivate":0,
				"device_id":"京A1671",
				"drive_status":2,
				"ele":4.8,
				"emergencyStatus":0,
				"hea":240.8,
				"lat":39.1234567,
				"lon":117.1234567,
				"outofControl":0,
				"pos_valid":true,
				"spd":2.376,
				"vehicle_num":"京A1671",
				"vehicle_type":10,
				"vip_status":0},
			"tag":2101
			}
	selfstat_json = json.dumps(selfstat)
	#initialize destination IP and port number
	client_port = 8088
	dest_ip = ''
	dest_port = 8080
	main()

