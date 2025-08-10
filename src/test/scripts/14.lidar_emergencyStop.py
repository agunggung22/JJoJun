"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

from math import *
import rospy
from std_msgs.msg import Float64
import os
from sensor_msgs.msg import LaserScan # ROS 이미지


class Turtle_sub:

    def __init__(self):
        rospy.init_node("sim_emergency_stop")
        self.pub = rospy.Publisher(
            "/commands/motor/speed", Float64, queue_size=1)
        self.speed_msg = Float64()

        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_CB)
        self.scan_msg = LaserScan()

    def lidar_CB(self, msg):
        os.system("clear")  # print를 깔끔하게

        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min*180/pi
        degree_max = self.scan_msg.angle_max*180/pi
        degree_increment = self.scan_msg.angle_increment*180/pi

        degrees = []
        degrees = [degree_min + degree_increment * index for index,
                   value in enumerate(self.scan_msg.ranges)]

        # sol1. for문을 쓰면, 속도가 덮어쓰이는 문제 발생
        # for index, value in enumerate(self.scan_msg.ranges):
        #     if -30 < degrees[index] < 30:
        #         if 0 < value < 1.5:
        #             print(f"{index} : {degrees[index]:.2f}")
        #             self.speed_msg.data=0
        #         else:
        #             self.speed_msg.data=1200

        # sol2. 한번이라도 장애물을 만나면 그 상태를 유지하는 방법
        self.speed_msg.data = 1200
        obstacle = 0
        for index, value in enumerate(self.scan_msg.ranges):
            if -30 < degrees[index] < 30 and 0 < value < 3:
                print(f"{index} : {degrees[index]:.2f}")
                obstacle = 1
        if obstacle == 1:
            self.speed_msg.data = 0

        self.pub.publish(self.speed_msg)


def main():
    try:
        turtle_sub = Turtle_sub()
        rospy.spin()
    except rospy.ROSInterruptException():  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
