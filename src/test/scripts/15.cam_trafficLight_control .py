"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

from morai_msgs.msg import GetTrafficLightStatus
from cv_bridge import CvBridge # openCV 이미지와 ROS 이미지를 변환
import numpy as np # 행렬 연산을 위한 라이브러리
import cv2
import rospy
from std_msgs.msg import Float64 # ROS 이미지


class Traffic_control:

    def __init__(self):
        rospy.init_node("Traffic_control")
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus,  self.traffic_CB)

        self.traffic_msg = GetTrafficLightStatus()
        self.traffic_flag = 0
        self.past_signal = 0

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)

        self.steer_msg = Float64()
        self.speed_msg = Float64()

    def traffic_CB(self, msg):
        # traffiLightStatus: red=1, yellow=4, green=16, left=33
        self.traffic_msg = msg

        if self.traffic_msg.trafficLightIndex == "SN000002":
            signal = self.traffic_msg.trafficLightStatus
            if self.past_signal != signal:
                self.traffic_flag = 0
                self.past_signal = 0

            self.steer_msg.data = 0.5
            self.traffic_flag += 1

            if signal == 1:  # stop
                self.speed_msg.data = 0
                print(f"red : {self.traffic_flag}")
            elif signal == 4:
                self.speed_msg.data = 800
                print(f"yellow : {self.traffic_flag}")
            elif signal == 16:
                self.speed_msg.data = 1600
                print(f"green : {self.traffic_flag}")
            elif signal == 33:  # 좌회전
                self.speed_msg.data = 800
                self.steer_msg.data = 0.3
                print(f"left : {self.traffic_flag}")
            else:
                print(f"{signal} : {self.traffic_flag}")
                self.speed_msg.data = 800
                self.steer_msg.data = 0.5

            self.speed_pub.publish(self.speed_msg)
            self.steer_msg.publish(self.steer_msg)
        else:
            pass


def main():
    try:
        traffic_control = Traffic_control()
        rospy.spin()
    except rospy.ROSInterruptException():  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
