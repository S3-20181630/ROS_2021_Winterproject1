#!/usr/bin/python
#-*- encoding: utf-8 -*-

import cv2, rospy, time
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from turtlesim.msg import Pose

bridge = CvBridge()
img = np.empty(shape=[0])
x, y, theta = 0, 0, 0

control_msg = Twist()

def image_callback(img_data):
    global bridge
    global img
    img = bridge.imgmsg_to_cv2(img_data, "bgr8")


def pose_callback(pose_data):
    global x
    global y
    global theta

    x = pose_data.x
    y = pose_data.y
    theta = pose_data.theta


def control_msg_publish(linear_x, angular_z):
    	control_msg.linear.x = linear_x
    	control_msg.linear.y = 0
    	control_msg.linear.z = 0
    	control_msg.angular.x = 0
    	control_msg.angular.y = 0
    	control_msg.angular.z = angular_z

        pub.publish(control_msg)

def go_destination(dest_x, dest_y):
	angular = math.atan2(dest_y-y, dest_x-x)
	distance = abs(math.sqrt((dest_x-x)**2 + (dest_y-y)**2))

	linear_speed = distance*0.6
	angular_speed = (angular - theta)*4.0

	print("publish", linear_speed, angular_speed)
	control_msg_publish(linear_speed, angular_speed)

	if (distance<0.7):
		control_msg_publish(0,0)


if __name__ == "__main__":
    rospy.init_node("foscar_project")
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

    time.sleep(1)
    

    while not rospy.is_shutdown():
        # 이미지 hsv 변환해서 저장하기
	lower_red = (0, 100, 100)
        upper_red = (10 , 255, 255)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	img_mask = cv2.inRange(img_hsv, lower_red, upper_red)
	img_result = cv2.bitwise_and(img, img, mask=img_mask)
	cv2.imshow("result",img_result)

	#화면에서 빨간색이 검출된 곳
	nonzero = np.nonzero(img_result)
	nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
	dest_x = np.sum(nonzerox) / len(nonzerox) *0.02
	dest_y = np.sum(nonzeroy) / len(nonzeroy) *-0.025 + 11
	print("dest_x, dest_y",dest_x, dest_y)

	# 거북이 제어
	if dest_x!=0 and dest_y!=0: #빨간색이 검출 되었을 때
		go_destination(dest_x,dest_y)
	else:
		print("Red is not detected")

	#control_msg_publish(distance,angular)


        # 거북이 pose 정보 출력
        print("x : ", x)
        print("y : ", y)
        print("theta : ", theta)
        print("")


        # 격자무늬 시각화
        cv2.line(img, (0, 160), (639, 160), (0, 0, 0), 1)
        cv2.line(img, (0, 320), (639, 320), (0, 0, 0), 1)
        cv2.line(img, (210, 0), (210, 480), (0, 0, 0), 1)
        cv2.line(img, (420, 0), (420, 480), (0, 0, 0), 1)

        # 비디오 화면 띄우기
        # 동시에 여러 창 띄우는것도 가능
        #cv2.imshow("image", img)
        #cv2.imshow("hsv_image", img_hsv)

        if cv2.waitKey(1) & 0xff == ord("q"):
            break

    cv2.destroyAllWindows()
