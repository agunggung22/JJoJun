"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

from  std_msgs.msg import Float64
from cv_bridge import CvBridge # openCV 이미지와 ROS 이미지를 변환
import numpy as np # 행렬 연산을 위한 라이브러리
import cv2
from sensor_msgs.msg import CompressedImage # ROS 이미지
import rospy


class Lane_sub:

    def __init__(self):
        rospy.init_node("lane_turn_node")
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB,
                         queue_size=1, buff_size=2**20, tcp_nodelay=True)
        self.ros_image = CompressedImage()
        self.bridge = CvBridge()

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_msg = Float64()
        self.speed_msg = Float64()
        self.speed_msg.data = 900.0   # ← 기본 주행 속도

        """# --- 1단계용: 교차로 감지용 변수 ---"""
        self.cross_flag = False  # 교차로(정지선) 감지 플래그
        self.cross_on_frames = 0  # 교차로에서의 프레임 수
        self.cross_off_frames = 0  # 교차로에서 벗어난 프레임 수

        self.x = 0  # 이미지 가로 크기
        self.y = 0  # 이미지 세로 크기

    def bird_view_transform(self, img):
        img = self.bridge.compressed_imgmsg_to_cv2(img)
        self.y, self.x = img.shape[0:2]

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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
        point1 = [0, self.y]  # 좌하단
        point2 = [283, 265]  # 좌상단
        point3 = [self.x-283, 265]  # 우상단
        point4 = [self.x, self.y]  # 우하단
        src_points = np.array([point1, point2, point3, point4], dtype=np.float32)

        # 2. 출력 영상에서 도로가 평행하도록 직사각형 좌표 지정(dst_points)
        dst_point1 = [self.x//4, self.y]
        dst_point2 = [self.x//4, 0]
        dst_point3 = [self.x//4*3, 0]
        dst_point4 = [self.x//4*3, self.y]
        dst_points = np.array([dst_point1, dst_point2, dst_point3, dst_point4], dtype=np.float32)

        # 3. 실제 변환
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        bird_view_img = cv2.warpPerspective(filtered_img, matrix, [self.x, self.y])

        # 4. 희끗한 부분을 아예 흰색으로 (이진화)
        grayed_img = cv2.cvtColor(bird_view_img, cv2.COLOR_RGB2GRAY)  # 흑백변환
        bin_img = np.zeros_like(grayed_img)  # 구조를 복사하되, 값은 0으로 채움
        bin_img[grayed_img > 80] = 1

        return bin_img

    def straight(self, img):
        bird_view_img = self.bird_view_transform(img)

        # 5. 차선의 중앙 추정
        histogram = np.sum(bird_view_img, axis=0)  # 차선이 어느 열에 더 많이 분포하는지 파악
        lane_indices = np.where(histogram > 22)[0]  # 각 차원별로 반환하므로 1차원 추출

        left_histogram = histogram[0:self.x//2]
        left_lane_indices = np.where(left_histogram > 22)[0]

        right_histogram = histogram[self.x//2:]
        right_lane_indices = np.where(right_histogram > 22)[0] + self.x//2
        print(len(left_lane_indices), len(right_lane_indices))

        try:
            if len(left_lane_indices) > 0 and len(right_lane_indices) > 0:
                center_index = (lane_indices[0]+lane_indices[-1])//2
                print("both line")
        except:
            center_index = self.x//2
            print("no lane")

       # 6. 차선의 중앙을 맞추는 PD 컨트롤  ← 이 블록 전체를 아래로 교체
        error = (center_index - self.x//2)  # 음수: 우측 치우침, 양수: 좌측 치우침

        # 이전 에러 준비
        if not hasattr(self, 'prev_error'):
            self.prev_error = 0.0
        normalized_error = float(error) / float(self.x)   # -0.5 ~ +0.5
        derivative = normalized_error - self.prev_error
        self.prev_error = normalized_error           # ← 업데이트 잊지 말기

        # [속도 기반 게인 가이드]
        #   Kp: "얼마나 세게 따라갈까?"  → 크면 빠르지만 불안정 가능
        #   Kd: "급변을 얼마나 누를까?" → 크면 진동 억제, 너무 크면 둔해짐
        # - 고속(속도 ↑): Kp ↓ (민감도 낮춰 안정성 ↑), Kd ↑ (오버슈트/진동 억제)
        # - 저속(속도 ↓): Kp ↑ (재빠른 추종), Kd ↓ (민첩성 유지)
        derivative *= 3.0  # Kd 효과 증폭 (값은 실험적으로 조정)
        speed_points = [600, 700, 900, 1200, 1500, 1800]
        Kp_points = [1.2, 1.0, 0.65, 0.55, 0.4, 0.3]
        Kd_points = [0.4, 0.5, 0.9, 1.0, 1.1, 1.2]
        v = float(self.speed_msg.data)

        Kp = np.interp(v, speed_points, Kp_points)
        Kd = np.interp(v, speed_points, Kd_points)

        # 조향 계산 + 클리핑
        steer_data = 0.5 + (Kp * normalized_error + Kd * derivative)
        return steer_data, self.speed_msg.data

    # 교차로/정지선 감지
    def stop_line_flag(self, img):
        bird_view_img = self.bird_view_transform(img)

        # 7. 가로선(정지선/교차로) 추출  ─ 하단부에서 '두꺼운 가로 밴드' 감지
        histogram_y = np.sum(bird_view_img, axis=1)     # y축 기준 -> 행의 합 히스토그램
        down_hist = histogram_y[self.y//2:]            # 하단 절반만 사용 (카메라 아래쪽)

        # 한 행(row)에 흰 픽셀 수가 화면 가로의 일정 비율 이상이면 '가로선 성분'으로 판단
        cross_row_pix_th = int(0.45 * self.x)          # 행당 흰 픽셀 임계치(가로 45%)  [튜닝 포인트]
        cross_rows = np.where(down_hist > cross_row_pix_th)[0]

        # '가로선'이 여러 행에 걸쳐 연속적으로 나타나면 교차로/정지선으로 판단
        cross_min_height = int(0.04 * self.y)          # 최소 두께(화면 높이의 4%)  [튜닝 포인트]

        detected_now = False
        try:
            if len(cross_rows) > 0:
                cross_span = cross_rows[-1] - cross_rows[0]   # 연속 구간 대략 두께
                if cross_span >= cross_min_height:
                    detected_now = True
                    # 시각화(버드뷰): 하단 오프셋(y//2)을 다시 더해 원래 좌표로 사각형 그림
                    p1 = (0, self.y//2 + int(cross_rows[0]))
                    p2 = (self.x-1, self.y//2 + int(cross_rows[-1]))
                    cv2.rectangle(bird_view_img, p1, p2, (0, 255, 0), 3)
        except:
            detected_now = False

        # 디바운싱(깜빡임 방지): 3프레임 이상 연속 검출 시 True, 3프레임 이상 연속 미검출 시 False
        if detected_now:
            self.cross_on_frames += 1
            self.cross_off_frames = 0
        else:
            self.cross_off_frames += 1
            self.cross_on_frames = 0

        if self.cross_on_frames >= 3:
            self.cross_flag = True
        elif self.cross_off_frames >= 3:
            self.cross_flag = False

        if self.cross_flag:
            print(">> 교차로/정지선 감지 (cross_flag = True)")
        return self.cross_flag

    def turn(self, img):
        """어려웡"""

    def cam_CB(self, msg):
        bird_view_img = self.bird_view_transform(msg)

        cv2.imshow("bird_view_img", bird_view_img)

        cv2.waitKey(1)


def main():
    try:
        turtle_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException():  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
