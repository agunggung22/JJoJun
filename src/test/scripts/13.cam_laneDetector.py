"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

import rospy
from sensor_msgs.msg import CompressedImage # ROS 이미지
import cv2
import numpy as np # 행렬 연산을 위한 라이브러리
from cv_bridge import CvBridge # openCV 이미지와 ROS 이미지를 변환
from  std_msgs.msg import Float64


class Lane_sub:

    def __init__(self):
        rospy.init_node("lane_sub_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.ros_image = CompressedImage()
        self.bridge = CvBridge()

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_msg = Float64()
        self.speed_msg = Float64()

        self.cross_flag = 0

    def cam_CB(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x = img.shape[0:2]

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)

        # 픽셀 값이 지정된 범위에 속하면 추출
        yellow_lower = np.array([15, 150, 0])  # 최소 임계값
        yellow_upper = np.array([40, 255, 255])  # 최대 임계값
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0, 0, 210])
        white_upper = np.array([179, 60, 255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_mask = cv2.bitwise_or(yellow_range, white_range)

        filtered_img = cv2.bitwise_and(img, img, mask=combined_mask)

        # 1. 입력 영상에서 도로의 사다리꼴 영역 지정(scr_points)
        point1 = [0, y]  # 좌하단
        point2 = [283, 265]  # 좌상단
        point3 = [x-283, 265]  # 우상단
        point4 = [x, y]  # 우하단
        src_points = np.array([point1, point2, point3, point4], dtype=np.float32)

        # 2. 출력 영상에서 도로가 평행하도록 직사각형 좌표 지정(dst_points)
        dst_point1 = [x//4, y]
        dst_point2 = [x//4, 0]
        dst_point3 = [x//4*3, 0]
        dst_point4 = [x//4*3, y]
        dst_points = np.array([dst_point1, dst_point2, dst_point3, dst_point4], dtype=np.float32)

        # 3. 실제 변환
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        bird_view_img = cv2.warpPerspective(filtered_img, matrix, [x, y])

        # 4. 희끗한 부분을 아예 흰색으로 (이진화)
        grayed_img = cv2.cvtColor(bird_view_img, cv2.COLOR_RGB2GRAY)  # 흑백변환
        bin_img = np.zeros_like(grayed_img)  # 구조를 복사하되, 값은 0으로 채움
        bin_img[grayed_img > 80] = 1
        bin2_img = np.zeros_like(grayed_img)  # 구조를 복사하되, 값은 0으로 채움
        bin2_img[grayed_img > 80] = 255

        # 5. 차선의 중앙 추정
        histogram = np.sum(bin_img, axis=0)  # 차선이 어느 열에 더 많이 분포하는지 파악
        lane_indices = np.where(histogram > 20)[0]  # 각 차원별로 반환하므로 1차원 추출

        left_histogram = histogram[0:x//2]
        left_lane_indices = np.where(left_histogram > 20)[0]

        right_histogram = histogram[x//2:]
        right_lane_indices = np.where(right_histogram > 20)[0] + 320

        try:
            if len(left_lane_indices) > 0 and len(right_lane_indices) == 0:  # 차가 오른쪽으로 기울어짐
                center_index = (
                    (left_lane_indices[0]+left_lane_indices[-1]))//2
                print("only left")

            elif len(left_lane_indices) == 0 and len(right_lane_indices) > 0:  # 차가 왼쪽으로 기울어짐
                center_index = (
                    (right_lane_indices[0]+right_lane_indices[-1]))//2
                print("only right")

            elif len(left_lane_indices) > 0 and len(right_lane_indices) > 0:
                center_index = (lane_indices[0]+lane_indices[-1])//2
                print("both line")
        except:
            center_index = x//2
            print("no lane")

         # 6. 차선의 중앙을 맞추는 PD 컨트롤  ← 이 블록 전체를 아래로 교체
        error = (center_index - x//2)  # 음수: 우측 치우침, 양수: 좌측 치우침

        # 이전 에러 준비
        if not hasattr(self, 'prev_error'):
            self.prev_error = 0.0
        normalized_error = float(error) / float(x)   # -0.5 ~ +0.5
        derivative = normalized_error - self.prev_error
        self.prev_error = normalized_error           # ← 업데이트 잊지 말기

        # 주행 속도 설정
        speed = 1000.0

        # [속도 기반 게인 가이드]
        #   Kp: "얼마나 세게 따라갈까?"  → 크면 빠르지만 불안정 가능
        #   Kd: "급변을 얼마나 누를까?" → 크면 진동 억제, 너무 크면 둔해짐
        # - 고속(속도 ↑): Kp ↓ (민감도 낮춰 안정성 ↑), Kd ↑ (오버슈트/진동 억제)
        # - 저속(속도 ↓): Kp ↑ (재빠른 추종), Kd ↓ (민첩성 유지)
        derivative *= 3.0  # Kd 효과 증폭 (값은 실험적으로 조정)
        speed_points = [600, 700, 900, 1200, 1500, 1800]
        Kp_points = [1.2, 1.0, 0.65, 0.55, 0.4, 0.3]
        Kd_points = [0.4, 0.5, 0.9, 1.0, 1.1, 1.2]
        v = float(speed)
        Kp = np.interp(v, speed_points, Kp_points)
        Kd = np.interp(v, speed_points, Kd_points)

        # 조향 계산 + 클리핑
        steer_data = 0.5 + (Kp * normalized_error + Kd * derivative)
        self.steer_msg.data = max(0.0, min(1.0, float(steer_data)))
        self.speed_msg.data = float(speed)
        print(f"steer: {self.steer_msg.data:.3f}, Kp:{Kp}, Kd:{Kd}, speed:{speed}")

        # 퍼블리시
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)

        # cv2.imshow("img", img)
        # cv2.imshow("img_yellow_range", yellow_range)
        # cv2.imshow("img_white_range", white_range)
        # cv2.imshow("combined_mask", combined_mask)
        cv2.imshow("bird_view_img", bird_view_img)
        # cv2.imshow("bin2_img", bin2_img)
        cv2.waitKey(1)


def main():
    try:
        turtle_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
