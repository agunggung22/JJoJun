# """#!: 쉐뱅 - 인터프리터 설정"""
# #!/usr/bin/env python3

# """# -*- coding: utf-8 -*- : 인코딩 설정"""
# # -*- coding:utf-8-*-

# from morai_msgs.msg import GetTrafficLightStatus
# from cv_bridge import CvBridge # openCV 이미지와 ROS 이미지를 변환
# import numpy as np # 행렬 연산을 위한 라이브러리
# import cv2
# import rospy
# from std_msgs.msg import Float64 # ROS 이미지


# class Traffic_control:

#     def __init__(self):
#         rospy.init_node("Traffic_control")
#         rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus,  self.traffic_CB)

#         self.traffic_msg = GetTrafficLightStatus()
#         self.traffic_flag = 0
#         self.past_signal = 0

#         self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
#         self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)

#         self.steer_msg = Float64()
#         self.speed_msg = Float64()

#     def traffic_CB(self, msg):
#         # traffiLightStatus: red=1, yellow=4, green=16, left=33
#         self.traffic_msg = msg

#         if self.traffic_msg.trafficLightIndex == "SN000002":
#             signal = self.traffic_msg.trafficLightStatus
#             if self.past_signal != signal:
#                 self.traffic_flag = 0
#                 self.past_signal = 0

#             self.steer_msg.data = 0.5
#             self.traffic_flag += 1

#             if signal == 1:  # stop
#                 self.speed_msg.data = 0
#                 print(f"red : {self.traffic_flag}")
#             elif signal == 4:
#                 self.speed_msg.data = 0
#                 print(f"yellow : {self.traffic_flag}")
#             elif signal == 16:
#                 self.speed_msg.data = 0
#                 print(f"green : {self.traffic_flag}")
#             elif signal == 33:  # 좌회전
#                 self.speed_msg.data = 800
#                 self.steer_msg.data = 0.3
#                 print(f"left : {self.traffic_flag}")
#             else:
#                 print(f"{signal} : {self.traffic_flag}")
#                 self.speed_msg.data = 800
#                 self.steer_msg.data = 0.5

#             self.speed_pub.publish(self.speed_msg)
#             self.steer_msg.publish(self.steer_msg)
#         else:
#             pass


# def main():
#     try:
#         traffic_control = Traffic_control()
#         rospy.spin()
#     except rospy.ROSInterruptException():  # ctrl C -> 강제종료
#         pass


# if __name__ == "__main__":
#     main()
#!/usr/bin/env python3
# -*- coding:utf-8-*-

import rospy
from std_msgs.msg import Float64
from morai_msgs.msg import GetTrafficLightStatus


class TrafficControl:
    def __init__(self):
        rospy.init_node("traffic_control")

        # 퍼블리셔
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_msg = Float64()
        self.speed_msg = Float64()

        # 상태
        self.target_index = "SN000009"   # 실제 들어오는 인덱스로 맞춤(필요 없으면 None)
        self.signal = None
        self.past_signal = None
        self.left_hold_ticks = 0         # 좌회전 유지 틱
        self.LEFT_HOLD_TICKS_MAX = 150   # 50Hz 기준 3초 유지
        self.desired_steer = 0.5
        self.desired_speed = 0

        # 구독
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.cb_tl)

        # 50Hz 주기로 계속 퍼블리시
        self.timer = rospy.Timer(rospy.Duration(0.02), self.cb_publish)

        rospy.loginfo("traffic_control started")

    def cb_tl(self, msg: GetTrafficLightStatus):
        rospy.loginfo_throttle(1.0, f"RX: index={msg.trafficLightIndex}, status={msg.trafficLightStatus}")

        if self.target_index and msg.trafficLightIndex != self.target_index:
            return

        self.signal = msg.trafficLightStatus  # red=1, yellow=4, green=16, left=33

        if self.past_signal != self.signal:
            self.past_signal = self.signal
            # 신호 변화 시 카운터 리셋
            self.left_hold_ticks = 0

        # 기본값(안전 정지)
        self.desired_steer = 0.5
        self.desired_speed = 0

        if self.signal == 1:      # red
            print("red")
            self.desired_speed = 0

        elif self.signal == 4:    # yellow
            print("yellow")
            self.desired_speed = 0

        elif self.signal == 16:   # green(직진)
            print("green")
            # 좌회전 유지 중이면 계속 좌로 보정
            if self.left_hold_ticks > 0:
                self.desired_steer = 0.3
            self.desired_speed = 800

        elif self.signal == 33:   # left arrow
            print("left")
            # 좌회전 각/속도 설정
            self.desired_steer = 0.3       # 필요시 더 강하게(예: 0.25)
            self.desired_speed = 800       # 너무 낮으면 체감이 약함
            # 좌회전 신호 감지시 유지 타이머 채우기
            self.left_hold_ticks = self.LEFT_HOLD_TICKS_MAX

        else:
            # 기타 신호 → 천천히 직진
            print(self.signal)
            self.desired_steer = 0.5
            self.desired_speed = 600

    def cb_publish(self, event):
        # 좌회전 유지 타이머 감소(그린 상태 등에서 지속적으로 꺾이게)
        if self.left_hold_ticks > 0:
            self.left_hold_ticks -= 1

        # 지속 퍼블리시
        self.steer_msg.data = float(max(0.0, min(1.0, self.desired_steer)))
        self.speed_msg.data = float(self.desired_speed)
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)


if __name__ == "__main__":
    try:
        TrafficControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
