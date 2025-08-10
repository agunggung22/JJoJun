# roslaunch rosbridge_server rosbridge_websocket.launch

"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

import rospy
from sensor_msgs.msg import CompressedImage # ROS 이미지
import cv2
import numpy as np # 행렬 연산을 위한 라이브러리
from cv_bridge import CvBridge # openCV 이미지와 ROS 이미지를 변환


class Lane_sub:

    def __init__(self):
        rospy.init_node("lane_sub_node")
        rospy.Subscriber("/image_jpeg/compressed",
                         CompressedImage, self.cam_CB)
        self.ros_image = CompressedImage()
        self.bridge = CvBridge()

    def cam_CB(self, msg):

        # blue = [255, 0, 0]
        # green = [0, 255, 0]
        # red = [0, 0, 255]
        # white = [255, 255, 255]

        # image = np.array([[blue, red, green, red, blue],
        #                   [red, blue, green, blue, red],
        #                   [blue, red, green, red, blue],
        #                   [red, blue, green, blue, red]], np.uint8)

        # print(image.shape)  # 행, 열, 채널
        # y, x = image.shape[0:2]
        # c = image.shape[2]

        # img = cv2.imread("img.jpg", cv2.IMREAD_COLOR)
        # cv2.namedWindow("img", cv2.WINDOW_NORMAL)  # 큰 크기로 보기
        # cv2.imshow("img", img)
        # cv2.waitKey(1)  # 딜레이 역할 + 이후 화면 갱신 -> 프레임 단위로 영상 출력 가능해짐
        # print(img.shape)

        zero = np.zeros([300, 300], np.uint8)  # 흑백 채널
        zero[0:155, :] = 255  # 0~154 의 행에, 전체 열 선택

        y, x = zero.shape[0:2]
        zero[y//3:y//3*2, :] = 128

        cv2.line(zero, [0, 0], [150, 150], [0, 0, 255], 3)
        cv2.rectangle(zero, [120, 160], [280, 320], [255, 0, 0], -1)
        cv2.imshow("zeros", zero)
        cv2.waitKey(0)


def main():
    try:
        turtle_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException():  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
