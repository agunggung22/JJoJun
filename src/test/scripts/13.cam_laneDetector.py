"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

from cv_bridge import CvBridge # openCV 이미지와 ROS 이미지를 변환
from  std_msgs.msg import Float64
import numpy as np # 행렬 연산을 위한 라이브러리
import cv2
from sensor_msgs.msg import CompressedImage # ROS 이미지
import rospy


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

        # # a. canny 엣지 이미지
        # canny_img = cv2.Canny(bin_img, 2, 2)

        # # b. 허프 변환
        # lines = cv2.HoughLinesP(canny_img, 0.01, np.pi / 180, 90, minLineLength=50, maxLineGap=5)
        # # c. 선 그리기
        # try:
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         cv2.line(bird_view_img, (x1, y1), (x2, y2), (0, 255, 0), 5)
        #         self.cross_flag += 1
        # except:
        #     pass

        # 6. 차선의 중앙을 맞추는 뱅뱅 컨트롤
        error = (center_index-x//2)  # 음수면 자동차가 오른쪽, 양수면 왼쪽으로 치우침
        nomalized_error = error/x  # -0.5 ~ +0.5
        steer_data = 0.5 + (nomalized_error)

        self.steer_msg.data = max(0, min(1, steer_data))  # 클리핑
        print(f"steer : {self.steer_msg.data}")

        self.speed_msg.data = 900
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
    except rospy.ROSInterruptException():  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
