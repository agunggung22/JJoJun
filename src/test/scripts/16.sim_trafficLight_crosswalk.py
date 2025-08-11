"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

from std_msgs.msg import Float64 # ROS 이미지
from time import * # 병목 찾기
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge # openCV 이미지와 ROS 이미지를 변환
from morai_msgs.msg import GetTrafficLightStatus
import rospy
import numpy as np # 행렬 연산을 위한 라이브러리
from math import *
import cv2


class Traffic_control:

    def __init__(self):
        rospy.init_node("Traffic_control")
        self.x = 0
        self.y = 0

        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cam_CB)
        self.ros_image = CompressedImage()
        self.bridge = CvBridge()
        self.cross_flag = 0
        self.img = None

        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus,  self.traffic_CB)
        self.traffic_msg = GetTrafficLightStatus()
        self.traffic_flag = 0
        self.signal = 0
        self.past_signal = 0

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.steer_msg = Float64()
        self.speed_msg = Float64()

        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_CB)
        self.scan_msg = LaserScan()

        self.steer_data = 0
        self.steer = 0.5
        self.obstacle_flag = False

    def traffic_CB(self, msg):
        # traffiLightStatus: red=1, yellow=4, green=16, left=33
        self.traffic_msg = msg

        if self.traffic_msg.trafficLightIndex == "SN000002":
            self.signal = self.traffic_msg.trafficLightStatus
            if self.past_signal != self. signal:
                self.past_signal = self.signal

            first_time = time()
            self.traffic_control()
            second_time = time()
            print(f"traffic delay : {second_time}-{first_time}")

    def traffic_control(self):
        if self.signal == 1:
            print("red")
        elif self.signal == 4:
            print("yellow")
        elif self.signal == 16:
            print("green")
        elif self.signal == 33:
            print("left")
        else:
            pass

    def cam_CB(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.steer_data = self.cam_lane_detection()

    def cam_lane_detection(self):
        self.y, self.x = self.img.shape[0:2]
        y, x = self.img.shape[0:2]

        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)

        # 픽셀 값이 지정된 범위에 속하면 추출
        yellow_lower = np.array([10, 100, 100])  # 최소 임계값
        yellow_upper = np.array([40, 255, 255])  # 최대 임계값
        yellow_range = cv2.inRange(img_hsv, yellow_lower, yellow_upper)

        white_lower = np.array([0, 0, 150])
        white_upper = np.array([179, 60, 255])
        white_range = cv2.inRange(img_hsv, white_lower, white_upper)

        combined_mask = cv2.bitwise_or(yellow_range, white_range)
        filtered_img = cv2.bitwise_and(self.img, self.img, mask=combined_mask)

        # 1. 입력 영상에서 도로의 사다리꼴 영역 지정(scr_points)
        point1 = [0, y]  # 좌하단
        point2 = [283, 265]  # 좌상단
        point3 = [x-283, 265]  # 우상단
        point4 = [x, y]  # 우하단
        src_points = np.array(
            [point1, point2, point3, point4], dtype=np.float32)

        # 2. 출력 영상에서 도로가 평행하도록 직사각형 좌표 지정(dst_points)
        dst_point1 = [x//4, y]
        dst_point2 = [x//4, 0]
        dst_point4 = [x//4*3, y]
        dst_point3 = [x//4*3, 0]
        dst_points = np.array(
            [dst_point1, dst_point2, dst_point3, dst_point4], dtype=np.float32)

        # 3. 실제 변환
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        bird_view_img = cv2.warpPerspective(filtered_img, matrix, [x, y])

        # 4. 희끗한 부분을 아예 흰색으로 (이진화)
        grayed_img = cv2.cvtColor(bird_view_img, cv2.COLOR_RGB2GRAY)  # 흑백변환
        bin_img = np.zeros_like(grayed_img)  # 구조를 복사하되, 값은 0으로 채움
        bin_img[grayed_img > 40] = 1

        # 5. 차선의 중앙 추정
        histogram_x = np.sum(bin_img, axis=0)  # x축 기준 -> 열의 합 히스토그램
        left_hist = histogram_x[0:x//2]
        left_lane_indices = np.where(left_hist > 20)[0]
        right_hist = histogram_x[x//2:]
        right_lane_indices = np.where(right_hist > 20)[0] + 320

        lane_indices = np.where(histogram_x > 20)[0]  # 각 차원별로 반환하므로 1차원 추출

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

        # 7. 가로선 추출
        histogram_y = np.sum(bin_img, axis=1)  # y축 기준 -> 행의 합 히스토그램
        up_hist = histogram_y[0:y//2]
        down_hist = histogram_y[y//2:]

        cross_indices = np.where(down_hist > 300)[0]
        try:
            if len(cross_indices) > 0:
                cross_length = cross_indices[-1]-cross_indices[0]
                cross_threshold = 40
                if cross_threshold < cross_length:
                    self.cross_flag = True
                    cv2.rectangle(bird_view_img, [0, cross_indices[0]], [x, cross_indices[-1]], (0, 255, 0), 3)
            else:
                self.cross_flag = False
        except:
            self.cross_flag = False

        # 6. 차선의 중앙을 맞추는 뱅뱅 컨트롤
        error = (center_index-x//2)  # 음수면 자동차가 오른쪽, 양수면 왼쪽으로 치우침
        nomalized_error = error/x  # -0.5 ~ +0.5
        steer_data = 0.5 + (nomalized_error)

        cv2.imshow("img", self.img)
        # cv2.imshow("img_yellow_range", yellow_range)
        # cv2.imshow("img_white_range", white_range)
        # cv2.imshow("combined_mask", combined_mask)
        cv2.imshow("bird_view_img", bird_view_img)
        cv2.waitKey(1)

        return steer_data

    def lidar_CB(self, msg):
        # os.system("clear")  # print를 깔끔하게
        self.scan_msg = msg
        self.steer = self.obstacle()

    def obstacle(self, msg):
        """ degree : -180 부터 180까지 360도임 """
        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min*180/pi
        degree_max = self.scan_msg.angle_max*180/pi
        degree_increment = self.scan_msg.angle_increment*180/pi  # 0.5 -> 723개
        # print(degree_min)
        # print(degree_max)
        # print(degree_increment)
        # print(len(self.scan_msg.ranges))

        degrees = np.array([degree_min + degree_increment * index for index,
                            value in enumerate(self.scan_msg.ranges)])
        # print(np.where((np.abs(degrees) > 89.7) & (np.abs(degrees) < 90.3))[0])
        # index 0 : -180
        # index 180 : -90
        # index 361 : 0
        # index 542 : +90
        # index 722 : 180

        obstacle_degrees = []
        obstacle_index = []
        for index, value in enumerate(self.scan_msg.ranges):
            if 0 < value < 1:
                # print(f"{index} : {degrees[index]:.2f}")
                obstacle_degrees.append(float(f"{degrees[index]:.2f}"))
                obstacle_index.append(index)
            else:
                pass
        # print(len(obstacle_degrees))
        # print(obstacle_degrees)
        print(obstacle_index)

        # 0 -> -19.5
        # 0.5 -> 0
        # 1 -> 19.5
        self.steer_msg.data = 0  # -90 ~ 90

        try:
            right_space = obstacle_index[0]-180
            left_space = 542 - obstacle_index[-1]
            # 뒤에 있으면 더이상 핸들을 꺾지 않도록하는 구문. 보완
            # if (right_space < 50 or left_space < 50) and not len(np.where(obstacle_index == 361)[0]):
            #     self.steer_msg.data = 0.5
            #     self.speed_msg.data = 1000
            if left_space < right_space:  # 장애물이 왼쪽에 위치
                right_index_avg = (obstacle_degrees[0]-90)/2
                steer = (abs(right_index_avg))/180+0.5

            else:   # 장애물이 오른쪽에 위치
                left_index_avg = (obstacle_degrees[-1]+90)/2
                steer = -(abs(left_index_avg)/180) + 0.5

            self.obstacle_flag = True

        except:  # 장애물이 없음
            steer = 0.5
            self.speed_msg.data = 1000
            self.obstacle_flag = False

        print(steer)
        return steer

        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)

    def action(self):
        if len(self.img) != 0:
            if self.cross_flag == True and self.signal == 1:
                speed = 0
                steer = 0.5
            else:
                if self.obstacle_flag == True:
                    steer = self.steer
                    speed = 500
                else:
                    steer = self.steer_data
                    speed = 1000

            self.steer_msg.data = max(0, min(1, steer))  # 클리핑
            print(f"steer : {self.steer_msg.data}")
            self.speed_msg.data = speed

            self.steer_pub.publish(self.steer_msg)
            self.speed_pub.publish(self.speed_msg)


def main():
    try:
        traffic_control = Traffic_control()
        while not rospy.is_shutdown():
            traffic_control.action()
        # rospy.spin() : pub 있는 노드에선 불필요
    except rospy.ROSInterruptException():  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
