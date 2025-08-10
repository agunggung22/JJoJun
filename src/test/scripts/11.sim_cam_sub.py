"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

from sensor_msgs.msg import CompressedImage # ROS 이미지
import rospy
import cv2
from cv_bridge import CvBridge # openCV 이미지와 ROS 이미지를 변환


class Turtle_sub:

    def __init__(self):
        rospy.init_node("trotle_sub_node")
        rospy.Subscriber("/image_jpeg/compressed",
                         CompressedImage, self.cam_CB)
        self.ros_image = CompressedImage()
        self.bridge = CvBridge()

    def cam_CB(self, msg):
        self.ros_image = msg
        cv_image = self.bridge.compressed_imgmsg_to_cv2(self.ros_image)
        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)  # 이미지 출력 창이 갱신되고 유지되도록 함
        # print("cam")


def main():
    try:
        turtle_sub = Turtle_sub()
        rospy.spin()
    except rospy.ROSInterruptException():  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
