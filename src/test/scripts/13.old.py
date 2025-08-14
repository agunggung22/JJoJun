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

        # 기본 주행 속도 설정
        self.default_speed = 1000.0

        self.cross_flag = 0

        # 1초 직진 관련 변수들
        self.straight_1sec_active = False
        self.straight_1sec_start_time = 0.0

    def straight_1sec(self, speed=None):
        """1초 동안 steer 0.5로 직진 주행하는 함수"""
        if speed is None:
            speed = self.default_speed

        current_time = rospy.get_time()

        # 처음 호출 시 시작 시간 기록
        if not self.straight_1sec_active:
            self.straight_1sec_active = True
            self.straight_1sec_start_time = current_time
            print("1초 직진 시작")

        # 1초가 지났는지 확인
        elapsed_time = current_time - self.straight_1sec_start_time

        if elapsed_time <= 1.0:
            # 1초 동안 직진
            steer = 0.5
            print(f"1초 직진 중: {elapsed_time:.2f}초 경과")
            return steer, speed, True  # 활성 상태
        else:
            # 1초 완료
            self.straight_1sec_active = False
            print("1초 직진 완료")
            return 0.5, speed, False  # 비활성 상태

    def bird_view_transform(self, img):
        """이미지를 버드뷰로 변환하는 함수"""
        img = self.bridge.compressed_imgmsg_to_cv2(img)
        y, x = img.shape[0:2]

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 픽셀 값이 지정된 범위에 속하면 추출
        yellow_lower = np.array([15, 150, 0])
        yellow_upper = np.array([40, 255, 255])
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0, 0, 210])
        white_upper = np.array([179, 60, 255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_mask = cv2.bitwise_or(yellow_range, white_range)
        filtered_img = cv2.bitwise_and(img, img, mask=combined_mask)

        # 원근 변환
        point1 = [0, y]
        point2 = [283, 265]
        point3 = [x-283, 265]
        point4 = [x, y]
        src_points = np.array([point1, point2, point3, point4], dtype=np.float32)

        dst_point1 = [x//4, y]
        dst_point2 = [x//4, 0]
        dst_point3 = [x//4*3, 0]
        dst_point4 = [x//4*3, y]
        dst_points = np.array([dst_point1, dst_point2, dst_point3, dst_point4], dtype=np.float32)

        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        bird_view_img = cv2.warpPerspective(filtered_img, matrix, [x, y])

        # 이진화
        grayed_img = cv2.cvtColor(bird_view_img, cv2.COLOR_RGB2GRAY)
        bin_img = np.zeros_like(grayed_img)
        bin_img[grayed_img > 80] = 1

        return bin_img, bird_view_img, x, y

    def get_center_index(self, img):
        """차선의 중앙점을 찾는 함수"""
        bin_img, _, x, y = self.bird_view_transform(img)

        # 히스토그램으로 차선 검detect
        histogram = np.sum(bin_img, axis=0)
        lane_indices = np.where(histogram > 20)[0]

        left_histogram = histogram[0:x//2]
        left_lane_indices = np.where(left_histogram > 20)[0]

        right_histogram = histogram[x//2:]
        right_lane_indices = np.where(right_histogram > 20)[0] + x//2

        try:
            if len(left_lane_indices) > 0 and len(right_lane_indices) == 0:
                center_index = (left_lane_indices[0] + left_lane_indices[-1]) // 2
                print("only left")
            elif len(left_lane_indices) == 0 and len(right_lane_indices) > 0:
                center_index = (right_lane_indices[0] + right_lane_indices[-1]) // 2
                print("only right")
            elif len(left_lane_indices) > 0 and len(right_lane_indices) > 0:
                center_index = (lane_indices[0] + lane_indices[-1]) // 2
                print("both line")
            else:
                center_index = x//2
                print("no lane")
        except:
            center_index = x//2
            print("no lane")

        return center_index, x

    def straight(self, center_index, x):
        """PD 제어를 이용한 직진 주행 함수"""
        error = (center_index - x//2)

        # 이전 에러 준비
        if not hasattr(self, 'prev_error'):
            self.prev_error = 0.0
        normalized_error = float(error) / float(x)
        derivative = normalized_error - self.prev_error
        self.prev_error = normalized_error

        # 주행 속도 설정
        speed = self.default_speed

        # PD 제어 게인 설정
        derivative *= 3.0
        speed_points = [600, 700, 900, 1200, 1500, 1800]
        Kp_points = [1.2, 1.0, 0.65, 0.55, 0.4, 0.3]
        Kd_points = [0.4, 0.5, 0.9, 1.0, 1.1, 1.2]
        v = float(speed)
        Kp = np.interp(v, speed_points, Kp_points)
        Kd = np.interp(v, speed_points, Kd_points)

        # 조향 계산
        steer_data = 0.5 + (Kp * normalized_error + Kd * derivative)
        return steer_data, speed, Kp, Kd

    def cam_CB(self, msg):
        """메인 콜백 함수"""
        # 1초 직진 모드 체크
        if self.straight_1sec_active:
            steer, speed, is_active = self.straight_1sec()
            if is_active:
                self.steer_msg.data = max(0.0, min(1.0, float(steer)))
                self.speed_msg.data = float(speed)
                self.steer_pub.publish(self.steer_msg)
                self.speed_pub.publish(self.speed_msg)
                return

        # 일반 차선 추종 모드
        center_index, x = self.get_center_index(msg)
        steer_data, speed, Kp, Kd = self.straight(center_index, x)

        # 안전 범위 클리핑
        self.steer_msg.data = max(0.0, min(1.0, float(steer_data)))
        self.speed_msg.data = float(speed)

        print(f"steer: {self.steer_msg.data:.3f}, Kp:{Kp}, Kd:{Kd}, speed:{speed}")

        # 퍼블리시
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)

        # 디버그 영상 출력
        _, bird_view_img, _, _ = self.bird_view_transform(msg)
        cv2.imshow("bird_view_img", bird_view_img)
        cv2.waitKey(1)


def main():
    try:
        turtle_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
