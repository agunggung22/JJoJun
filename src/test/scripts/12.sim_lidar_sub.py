"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

import os
from sensor_msgs.msg import LaserScan # ROS 이미지
import rospy
from math import *


class Turtle_sub:

    def __init__(self):
        rospy.init_node("trotle_sub_node")
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_CB)
        self.scan_msg = LaserScan()

    def lidar_CB(self, msg):
        os.system("clear")  # print를 깔끔하게

        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min*180/pi
        degree_max = self.scan_msg.angle_max*180/pi
        # print(degree_min)
        # print(degree_max)

        degree_increment = self.scan_msg.angle_increment*180/pi

        # range의 index를 각도로 변환
        degrees = []

        # sol1.
        # for index, value in enumerate(self.scam_msg.ranges):  # -180 부터 179까지 360도임
        #     degree = degree_min + degree_increment * index
        #     degrees.append(degree)
        # print(degrees)

        # sol2. [표현식 for 변수 in 반복가능한객체 if 조건]
        degrees = [degree_min + degree_increment * index for index,
                   value in enumerate(self.scan_msg.ranges)]

        for index, value in enumerate(self.scan_msg.ranges):
            if -30 < degrees[index] < 30 and 0 < value < 1.5:
                print(f"{index} : {degrees[index]:.2f}")


def main():
    try:
        turtle_sub = Turtle_sub()
        rospy.spin()
    except rospy.ROSInterruptException():  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
