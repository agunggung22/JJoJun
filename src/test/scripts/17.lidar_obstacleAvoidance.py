"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3
"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

import os
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan # ROS 이미지
from math import *
from std_msgs.msg import Float64


class Turtle_sub:

    def __init__(self):
        rospy.init_node("sim_emergency_stop")
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.speed_msg = Float64()

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.steer_msg = Float64()

        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_CB)
        self.scan_msg = LaserScan()

    """ Lidar 데이터 처리 """

    def process_lidar(self, msg):
        degree_min = msg.angle_min * 180 / pi
        degree_increment = msg.angle_increment * 180 / pi
        degrees = np.array([degree_min + degree_increment * i for i in range(len(msg.ranges))])
        return degrees

    """ 장애물 탐지 """
    # half_fov: 좌우 45도 이내만 탐지, 필요시 30~60으로 조정
    # dist_thresh: 장애물 거리 임계값, 필요시 0.5~2.0으로 조정

    def detect_obstacles(self, ranges, degrees, half_fov=45.0, dist_thresh=1.0):
        obstacle_degrees = []
        obstacle_index = []
        for idx, val in enumerate(ranges):
            if abs(degrees[idx]) <= half_fov and 0.0 < val < dist_thresh:
                obstacle_degrees.append(degrees[idx])
                obstacle_index.append(idx)
        return obstacle_degrees, obstacle_index

    """ 장애물 회피 알고리즘 """

    def avoidance_steer(self, obstacle_degrees, obstacle_index):
        if obstacle_index:
            right_space = obstacle_index[0] - 180
            left_space = 542 - obstacle_index[-1]
            if right_space > left_space:
                degree_avg = (obstacle_degrees[0] - 90) / 2
            else:
                degree_avg = (obstacle_degrees[-1] + 90) / 2
            steer = ((-degree_avg / 90) + 0.5)
        else:
            steer = 0.5
        return steer

    """ 명령어 발행 """

    def publish_cmd(self, speed, steer):
        self.speed_msg.data = speed
        self.steer_msg.data = steer
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)

    """ Lidar 콜백 함수 """

    def lidar_CB(self, msg):
        degrees = self.process_lidar(msg)
        obstacle_degrees, obstacle_index = self.detect_obstacles(msg.ranges, degrees)
        steer = self.avoidance_steer(obstacle_degrees, obstacle_index)
        self.publish_cmd(1000.0, steer)
        print(f"Steer: {steer}, Obstacles: {obstacle_degrees}")


def main():
    try:
        turtle_sub = Turtle_sub()
        rospy.spin()
    except rospy.ROSInterruptException:  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
