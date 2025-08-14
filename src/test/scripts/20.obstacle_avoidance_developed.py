#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from math import *
import os
import numpy as np

class Turtle_sub:
    def __init__(self):
        rospy.init_node("sim_e_stop_node")
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_CB)
        self.scan_msg = LaserScan()
        self.speed_msg = Float64()
        self.steer_msg = Float64()
        self.lane_changed = False  # 차선 변경 여부
        self.lane_returning = False  # 원래 차선 복귀 여부

    def lidar_CB(self, msg):
        os.system("clear")
        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min * 180 / pi
        degree_max = self.scan_msg.angle_max * 180 / pi
        degree_angle_increment = self.scan_msg.angle_increment * 180 / pi

        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)]
        obstacle_front = []
        obstacle_rear = []

        # 장애물 인식 개선: 더 넓은 각도, 더 넓은 거리 범위 사용
        for index, value in enumerate(self.scan_msg.ranges):
            # 전방 장애물: -45~45도, 0.05~1.5m
            if -45 <= degrees[index] <= 45 and 0.05 < value < 1.5:
                obstacle_front.append(index)
            # 후방 장애물: 135~180도 및 -135~-180도, 0.05~1.5m
            if (135 <= abs(degrees[index]) <= 180) and 0.05 < value < 1.5:
                obstacle_rear.append(index)

        # 차선 검출 (예시)
        white_left = any(0 < self.scan_msg.ranges[i] < 2 for i, deg in enumerate(degrees) if -80 <= deg <= -60)
        white_right = any(0 < self.scan_msg.ranges[i] < 2 for i, deg in enumerate(degrees) if 60 <= deg <= 80)
        yellow_center = any(0 < self.scan_msg.ranges[i] < 2 for i, deg in enumerate(degrees) if -10 <= deg <= 10)

        steer = 0.5
        speed = 1000  # 기본 속도

        # 장애물 회피 로직
        if obstacle_front and not self.lane_changed:
            # 흰색 차선 위치에 따라 반대 차선으로 회피
            if white_left and not yellow_center:
                steer = 0.8  # 오른쪽으로 회피 (왼쪽 흰선 감지)
                print("왼쪽 흰색 차선 감지: 오른쪽 차선으로 회피")
            elif white_right and not yellow_center:
                steer = 0.2  # 왼쪽으로 회피 (오른쪽 흰선 감지)
                print("오른쪽 흰색 차선 감지: 왼쪽 차선으로 회피")
            elif yellow_center:
                steer = 0.5  # 중앙선(노란선) 침범 금지, 직진 유지
                print("노란선 감지: 중앙선 침범 금지, 직진")
            else:
                steer = 0.8  # 기본적으로 오른쪽 회피
                print("흰색 차선 미검출: 기본 오른쪽 회피")
            speed = 400  # 회피 시 속도 크게 감소
            self.lane_changed = True
            self.lane_returning = False
        elif self.lane_changed and obstacle_rear and not self.lane_returning:
            # 원래 차선 복귀 (노란선 침범 금지)
            if yellow_center:
                steer = 0.5  # 중앙선 침범 금지, 직진
                print("노란선 감지: 중앙선 침범 금지, 직진 복귀 대기")
            else:
                if white_left:
                    steer = 0.2  # 왼쪽으로 복귀
                    print("왼쪽 흰색 차선 감지: 왼쪽 차선으로 복귀")
                elif white_right:
                    steer = 0.8  # 오른쪽으로 복귀
                    print("오른쪽 흰색 차선 감지: 오른쪽 차선으로 복귀")
                else:
                    steer = 0.5  # 차선 미검출 시 직진
                    print("차선 미검출: 직진 복귀")
                speed = 400  # 복귀 시에도 속도 유지
                self.lane_returning = True
        elif self.lane_returning and not obstacle_rear:
            steer = 0.5
            speed = 1000  # 복귀 후 정상 속도
            self.lane_changed = False
            self.lane_returning = False
            print("후방 장애물 없음: 직진")
        else:
            steer = 0.5
            speed = 1000

        print(f"steer: {steer}, speed: {speed}")
        self.speed_msg.data = speed
        self.steer_msg.data = steer
        self.speed_pub.publish(self.speed_msg)
        self.steer_pub.publish(self.steer_msg)

def main():
    try:
        turtle_sub = Turtle_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()