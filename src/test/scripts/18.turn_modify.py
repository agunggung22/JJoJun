"""#!: 쉐뱅 - 인터프리터 설정"""
#!/usr/bin/env python3

"""# -*- coding: utf-8 -*- : 인코딩 설정"""
# -*- coding:utf-8-*-

import rospy
from sensor_msgs.msg import CompressedImage # ROS 이미지
import cv2
import numpy as np # 행렬 연산을 위한 라이브러리
from  std_msgs.msg import Float64
from cv_bridge import CvBridge # openCV 이미지와 ROS 이미지를 변환
from sensor_msgs.msg import LaserScan


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

        self.speed_msg.data = 1000.0   # ← 기본 주행 속도 초기화
        self.speed = 1000.0   # ← 기본 주행 속도
        self.count = 0
        # 기본 차선폭은 320

        """# --- 1단계용: 교차로 감지용 변수 ---"""
        self.cross_flag = False  # 교차로(정지선) 감지 플래그
        self.cross_on_frames = 0  # 교차로에서의 프레임 수
        self.cross_off_frames = 0  # 교차로에서 벗어난 프레임 수

        # 회전 관련 변수들
        self.turning = False
        self.turn_dir = "left"  # "left" 또는 "right"
        self.turn_phase = "idle"
        self.turn_t0 = 0.0
        self.turn_speed = 1000  # 회전 시 고정 속도
        self.turn_speed2 = 500.0  # 회전 시 고정 속도

        # 차선 한 개일 경우 변수들
        self.one_lane_sequence = ['right', 'right', 'right', 'left']  # one_lane 시퀀스
        self.one_lane_index = 0  # one_lane 시퀀스 인덱스
        self.one_lane_flag = False  # 차선 한 개일 때 플래그
        self.straight_start_time = 0.0  # straight 시작 시간

        # 시퀀스 관련 변수들
        # self.action_sequence = ["left", "left", "left", "left"]  # 동작 시퀀스
        self.action_sequence = ["straight", "straight", "left", "straight"]  # 동작 시퀀스
        self.current_seq_index = 0  # 현재 시퀀스 인덱스
        self.stop_line_detected_time = 0.0  # 정지선 마지막 감지 시간
        self.stop_line_ignore_duration = 2.0  # 정지선 무시 시간 (초)
        self.stop_line_triggered = False  # 정지선 트리거 플래그

        # 정지선 후 직진 유지 관련 변수들
        self.post_stopline_straight = False  # 정지선 후 직진 유지 플래그
        self.post_stopline_start_time = 0.0  # 정지선 후 직진 시작 시간
        self.post_stopline_duration = 0.8  # 정지선 후 직진 유지 시간 (초)
        self.pending_action = None  # 대기 중인 동작

        # 4번째 정지선 후 속도 제어 변수들
        self.fourth_stopline_passed = False  # 4번째 정지선 통과 플래그
        self.fourth_stopline_time = 0.0  # 4번째 정지선 통과 시간

        # straight() 함수용 변수들 추가
        self.prev_error = 0.0
        self.prev_steer = 0.5
        self.prev_time = 0.0  # 추가: 이전 시간 저장용

        self.x = 0  # 이미지 가로 크기
        self.y = 0  # 이미지 세로 크기

        # LiDAR 구독
        rospy.Subscriber("/lidar2D", LaserScan, self.lidar_cb, queue_size=1)
        self.scan_msg = LaserScan()

        # 동적 장애물 상태기계
        self.obs_state = "clear"        # ["clear","wait"]
        self.obs_first_stop_t = None    # 정지 시작 시각 (10s 초과 방지 확인용)
        self.obs_last_seen_t = 0.0      # 마지막으로 장애물을 감지한 시각
        self.obs_centered = False       # 차로 중앙부에 있었는지
        # 튜닝 파라미터
        self.obs_front_deg = 30.0       # 전방 섹터(±deg)
        self.obs_detect_dist = 2     # 장애물 탐지 거리(m)
        self.lane_half_width = 1.75     # 차로 반폭 추정(필요시 조정)
        self.edge_margin = 0.40         # 가장자리 판정 여유(m)
        self.resume_hold = 0.6          # 재출발 전 추가 확인 시간(s)
        self.obs_edge_since = 0.0

        # 정적 장애물 회피 상태
        self.avoiding_static = False
        self.avoid_start_time = 0.0
        self.avoid_dir = "left"
        self.recovered = False

        # 정적 판단 파라미터
        self.static_detect_dist = 2.0   # 정면 r ≤ 2m 이내면 후보
        self.static_persist = 0.5       # 0.5s 이상 같은 자리면 '정적'
        self.static_seen_since = 0.0
        self.static_last_r = None
        self.static_last_lat = None

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

    def get_center_index(self, img, direction=None):
        bird_view_img = self.bird_view_transform(img)

        # 하단 6~10% 마스킹: 바로 앞(정지선/노이즈) 영향 줄이고 look-ahead 강화
        guard = max(30, int(0.08 * self.y))
        bird_view_img[self.y-guard:self.y, :] = 0

        # 5. 차선의 중앙 추정
        histogram = np.sum(bird_view_img, axis=0)  # 차선이 어느 열에 더 많이 분포하는지 파악
        lane_indices = np.where(histogram > 20)[0]  # 각 차원별로 반환하므로 1차원 추출

        left_histogram = histogram[0:self.x//2]
        left_lane_indices = np.where(left_histogram > 20)[0]

        right_histogram = histogram[self.x//2:]
        right_lane_indices = np.where(right_histogram > 20)[0] + self.x//2

        try:
            if direction is None:
                if len(left_lane_indices) > 0 and len(right_lane_indices) > 0:
                    center_index = (lane_indices[0] + lane_indices[-1]) // 2
                    # print("both line")
                elif len(left_lane_indices) > 0:
                    # 왼쪽만 보이면 오른쪽을 추정(차선폭 픽셀은 트랙 기준 고정/튜닝값)
                    lane_w = getattr(self, 'lane_w_px', int(0.30*self.x))
                    center_index = int((left_lane_indices.mean() + lane_w/2))
                elif len(right_lane_indices) > 0:
                    lane_w = getattr(self, 'lane_w_px', int(0.30*self.x))
                    center_index = int((right_lane_indices.mean() - lane_w/2))
                else:
                    center_index = self.x//2
            elif direction == "left":
                if len(left_lane_indices) > 0:
                    center_index = (left_lane_indices[0] + left_lane_indices[-1]) // 2 + 315/2
                else:
                    center_index = self.x//2
            elif direction == "right":
                if len(right_lane_indices) > 0:
                    center_index = (right_lane_indices[0] + right_lane_indices[-1]) // 2 - 315/2
                else:
                    center_index = self.x//2

        except Exception as e:
            center_index = self.x//2

        # ====== 여기 2줄 추가: keep-right 편향 ======
        keep_right_floor = int(self.x * 0.58)   # 0.56~0.60 사이 튜닝
        center_index = max(center_index, keep_right_floor)
        # =========================================

        return center_index

    def straight(self, center_index):
        """13.old 기반 고급 PD 제어로 진동 억제 + 코너 추종 향상"""
        error = (center_index - self.x//2)
        normalized_error = float(error) / float(self.x)   # -0.5 ~ +0.5

        # 미분 계산 (진동 억제용)
        derivative = normalized_error - self.prev_error
        self.prev_error = normalized_error

        # 13.old와 동일한 PD 게인 설정
        derivative *= 3.0  # Kd 효과 증폭
        speed_points = [600, 700, 900, 1200, 1500, 1800]
        Kp_points = [1.2, 1.0, 0.65, 0.55, 0.4, 0.3]
        Kd_points = [0.4, 0.5, 0.9, 1.0, 1.1, 1.2]

        v = float(self.speed_msg.data)

        Kp = np.interp(v, speed_points, Kp_points)
        Kd = np.interp(v, speed_points, Kd_points)

        # 코너 추종 향상을 위한 게인 부스트
        error_magnitude = abs(normalized_error)
        if error_magnitude > 0.1:  # 큰 에러일 때 (코너)
            Kp *= 1.3  # Kp 30% 증가

        # PD 제어 계산
        steer_data = 0.5 + (Kp * normalized_error + Kd * derivative)

        # 스티어 변화율 제한 (진동 억제)
        max_steer_change = 0.15  # 한 프레임당 최대 변화량
        if abs(steer_data - self.prev_steer) > max_steer_change:
            if steer_data > self.prev_steer:
                steer_data = self.prev_steer + max_steer_change
            else:
                steer_data = self.prev_steer - max_steer_change

        self.prev_steer = steer_data
        print(steer_data)

        return steer_data, self.speed

    # 교차로/정지선 감지

    def stop_line_flag(self, img):
        # 정지선 무시 시간 체크
        current_time = rospy.get_time()
        if current_time - self.stop_line_detected_time < self.stop_line_ignore_duration:
            return False  # 무시 시간 동안은 정지선 감지 안함

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
        except:
            detected_now = False

        # 디바운싱(깜빡임 방지): 3프레임 이상 연속 검출 시 True, 3프레임 이상 연속 미검출 시 False
        if detected_now:
            self.cross_on_frames += 1
            self.cross_off_frames = 0
        else:
            self.cross_off_frames += 1
            self.cross_on_frames = 0

        # 플래그 상태 업데이트
        if self.cross_on_frames >= 3:
            self.cross_flag = True
        elif self.cross_off_frames >= 3:
            self.cross_flag = False

        return self.cross_flag

    def turn(self, img, direction=None):
        """
        하드코딩 좌/우회전.
        - 신호/정지선 검출과 무관하게 'turning' 플래그가 켜진 상태에서 시간 기반으로 스티어를 내보냄
        - 종료 조건: (i) 회전 타임아웃 초과, (ii) 최소 유지 시간 후 차선 재획득 프레임 누적
        """
        if direction is None:
            direction = self.turn_dir

        now = rospy.get_time()
        # turning이 False일 때만 초기화 (한 번만 실행)
        if not self.turning:
            self.turn_phase = "enter"
            self.turn_t0 = now
            self.turning = True  # 회전 시작

        t = now - self.turn_t0

        # ===== 하드코딩 스티어 타임라인(필요하면 숫자만 조정) =====
        # 스티어 기준: 0.5가 직진, 0.3쪽이 좌, 0.7쪽이 우
        if direction == "left":
            # 0.0~0.5s : 약하게 좌로 코너 진입
            # 0.5~1.2s : 강하게 좌회전
            # 1.2~2.0s : 더 강하게 좌회전
            # 2.0~2.8s : 완만 좌(복원)
            if t < 0.5:
                steer = 0.4
            elif t < 1.2:
                steer = 0.3
            elif t < 2.0:
                steer = 0.22
            elif t < 2.8:
                steer = 0.3
            else:
                steer = 0.40  # 복원 대기
        else:  # right - 왼쪽 회전과 대칭으로 수정
            # 0.0~0.5s : 약하게 우로 코너 진입 (0.4 -> 0.6)
            # 0.5~1.2s : 강하게 우회전 (0.3 -> 0.7)
            # 1.2~2.0s : 더 강하게 우회전 (0.22 -> 0.78)
            # 2.0~2.8s : 완만 우(복원) (0.3 -> 0.7)
            if t < 0.5:
                steer = 0.6   # 0.4 대칭
            elif t < 1.2:
                steer = 0.7   # 0.3 대칭
            elif t < 2.0:
                steer = 0.78  # 0.22 대칭
            elif t < 2.8:
                steer = 0.7   # 0.3 대칭
            else:
                steer = 0.60  # 0.40 대칭

        v_out = self.turn_speed  # 회전 시 고정 속도 사용
        timeout = 3.2  # 필요 시 미세 조정
        need_exit = t > timeout

        if need_exit:
            # 종료 리셋
            self.turning = False
            self.turn_phase = "idle"

        return steer, v_out

    # def lidar_cb(self, msg: LaserScan):
    #     self.scan_msg = msg
    #     deg_min = msg.angle_min * 180.0 / np.pi
    #     deg_inc = msg.angle_increment * 180.0 / np.pi
    #     now = rospy.get_time()

    #     front_hits = []
    #     for i, r in enumerate(msg.ranges):
    #         if np.isfinite(r) and r > 0.03:
    #             th = deg_min + deg_inc * i
    #             if -self.obs_front_deg <= th <= self.obs_front_deg and r <= self.obs_detect_dist:
    #                 lat = r * np.sin(np.deg2rad(th))   # 횡방향 오프셋
    #                 front_hits.append((r, th, lat))

    #     hit = len(front_hits) > 0
    #     center_ok = edge_ok = False
    #     if hit:
    #         r, th, lat = min(front_hits, key=lambda x: x[0])
    #         center_ok = (abs(lat) < self.lane_half_width)
    #         edge_ok = (abs(lat) > (self.lane_half_width + self.edge_margin))

    #     if self.obs_state == "clear":
    #         if hit and center_ok:
    #             self.obs_state = "wait"
    #             self.obs_centered = True
    #             self.obs_first_stop_t = now
    #             self.obs_last_seen_t = now
    #             self.obs_edge_since = 0.0
    #             rospy.loginfo("[DYN-OBS] 중앙부 장애물 감지 → 정지 대기 진입")
    #     else:
    #         if hit:
    #             self.obs_last_seen_t = now
    #             # 가장자리에서 일정 시간 유지되면 재출발 후보
    #             if edge_ok:
    #                 if self.obs_edge_since == 0.0:
    #                     self.obs_edge_since = now
    #             else:
    #                 self.obs_edge_since = 0.0
    #         else:
    #             # 완전 소실
    #             self.obs_edge_since = 0.0

    def avoid_obstacle(self, msg):
        """정적 장애물 회피: 차선 변경 -> 유지 -> 복귀 (방향: self.avoid_dir)"""
        steer, speed = 0.5, self.speed
        t = rospy.get_time() - self.avoid_start_time
        left = (self.avoid_dir == "left")

        if t <= 1.0:
            steer = 0.3 if left else 0.7   # 변경
            speed = 800
            print(f"장애물 회피: {'좌' if left else '우'}측 변경")
        elif t <= 2.0:
            steer = 0.5                    # 유지
            speed = 800
            print("장애물 회피: 옆 차선 유지")
        elif t <= 3.0:
            steer = 0.7 if left else 0.3   # 복귀
            speed = 800
            print("장애물 회피: 원차선 복귀")
        else:
            self.recovered = True          # 한 사이클 종료

        return steer, speed

    def lidar_cb(self, msg: LaserScan):
        self.scan_msg = msg
        deg_min = msg.angle_min * 180.0 / np.pi
        deg_inc = msg.angle_increment * 180.0 / np.pi
        now = rospy.get_time()

        # ---------- 전방 히트 수집 (기존) ----------
        front_hits = []
        for i, r in enumerate(msg.ranges):
            if np.isfinite(r) and r > 0.03:
                th = deg_min + deg_inc * i
                if -self.obs_front_deg <= th <= self.obs_front_deg and r <= self.obs_detect_dist:
                    lat = r * np.sin(np.deg2rad(th))   # 횡 오프셋
                    front_hits.append((r, th, lat))

        hit = len(front_hits) > 0
        center_ok = edge_ok = False
        r_min = None
        lat_min = None
        if hit:
            r_min, th_min, lat_min = min(front_hits, key=lambda x: x[0])
            center_ok = (abs(lat_min) < self.lane_half_width)
            edge_ok = (abs(lat_min) > (self.lane_half_width + self.edge_margin))

        # ---------- (A) 정적 장애물 후보 판정 ----------
        # 정면 좁은 창(±8°)에서 r ≤ static_detect_dist 이고, 위치 변화가 거의 없으면 '정적'
        static_hit = False
        if r_min is not None and r_min <= self.static_detect_dist:
            # 지속성 체크: 처음 보면 시작, 이후엔 위치 변화량/시간으로 '정적성' 확인
            if self.static_seen_since == 0.0:
                self.static_seen_since = now
                self.static_last_r = r_min
                self.static_last_lat = lat_min
            else:
                dt = now - self.static_seen_since
                dr = abs(r_min - (self.static_last_r or r_min))
                dlat = abs(lat_min - (self.static_last_lat or lat_min))
                if dt >= self.static_persist and dr < 0.25 and dlat < 0.15:
                    static_hit = True
            # 최신값 갱신
            self.static_last_r = r_min
            self.static_last_lat = lat_min
        else:
            # 정면 깨끗하면 정적 후보 리셋
            self.static_seen_since = 0.0
            self.static_last_r = None
            self.static_last_lat = None

        # ---------- (B) 회피 방향 결정 (좌/우 전방 자유공간 비교) ----------
        if static_hit and (not self.avoiding_static) and (self.obs_state != "wait"):
            # 좌(+20~+60°), 우(-60~-20°)의 중앙값 거리 비교
            left_win = []
            right_win = []
            for i, r in enumerate(msg.ranges):
                if not np.isfinite(r) or r <= 0.03:
                    continue
                th = deg_min + deg_inc * i
                if 20.0 <= th <= 60.0:
                    left_win.append(r)
                if -60.0 <= th <= -20.0:
                    right_win.append(r)
            left_med = np.median(left_win) if left_win else 0.0
            right_med = np.median(right_win) if right_win else 0.0

            if left_med - right_med > 0.5:
                self.avoid_dir = "left"
            elif right_med - left_med > 0.5:
                self.avoid_dir = "right"
            else:
                self.avoid_dir = "left"   # 애매하면 좌로 기본

            self.avoiding_static = True
            self.avoid_start_time = now
            self.recovered = False
            rospy.loginfo(f"[STATIC-OBS] r={r_min:.2f}m, lat={lat_min:.2f}m → 회피 시작 ({self.avoid_dir})")
            return  # 회피 모드에 맡기기

        # ---------- (C) 동적 장애물 대기 로직 (수정) ----------
        # 정적 후보 ‘검증 중’(static_persist 미만)에는 wait로 진입하지 않도록 가드
        static_guard = (
            (r_min is not None) and
            (r_min <= self.static_detect_dist) and
            (self.static_seen_since > 0.0) and
            ((now - self.static_seen_since) < self.static_persist)
        )

        if self.obs_state == "clear":
            # 정적 회피 중/확정/검증중이면 대기 진입 금지
            if hit and center_ok and (not self.avoiding_static) and (not static_hit) and (not static_guard):
                self.obs_state = "wait"
                self.obs_centered = True
                self.obs_first_stop_t = now
                self.obs_last_seen_t = now
                self.obs_edge_since = 0.0
                rospy.loginfo("[DYN-OBS] 중앙부 장애물 감지 → 정지 대기 진입")
        else:
            if hit:
                self.obs_last_seen_t = now
                if edge_ok:
                    if self.obs_edge_since == 0.0:
                        self.obs_edge_since = now
                else:
                    self.obs_edge_since = 0.0
            else:
                self.obs_edge_since = 0.0

    def cam_CB(self, msg):
        steer = 0.5
        speed = self.speed

        prev_cross_flag = self.cross_flag
        self.stop_line_flag(msg)
        current_time = rospy.get_time()

        # === 정적 장애물 회피가 진행 중이면 최우선 적용 ===
        if self.avoiding_static:
            steer, speed = self.avoid_obstacle(msg)
            self.steer_msg.data = float(steer)
            self.speed_msg.data = float(speed)
            self.steer_pub.publish(self.steer_msg)
            self.speed_pub.publish(self.speed_msg)

            # (선택) 디버그 뷰가 필요하면 주석 해제
            # bird_view_img = self.bird_view_transform(msg) * 255
            # cv2.imshow("bird_view_img", bird_view_img); cv2.waitKey(1)

            if self.recovered:
                rospy.loginfo("✅ 회피 완료, 정상 주행 복귀")
                self.avoiding_static = False
                self.recovered = False
            return

        # === 동적 장애물: 대기 상태면 우선 정지 ===
        if self.obs_state == "wait":
            if self.obs_first_stop_t is not None and (current_time - self.obs_first_stop_t) > 9.5:
                rospy.logwarn("[DYN-OBS] 정지 10초 임박/초과!")

            # 정지 유지
            self.steer_msg.data = 0.5
            self.speed_msg.data = 0.0
            self.steer_pub.publish(self.steer_msg)
            self.speed_pub.publish(self.speed_msg)

            # 해제 조건: (가장자리에서 resume_hold 유지) 또는 (완전 소실 후 resume_hold 경과)
            edge_ok = (self.obs_edge_since > 0.0) and ((current_time - self.obs_edge_since) > self.resume_hold)
            lost_ok = (current_time - self.obs_last_seen_t) > self.resume_hold
            if edge_ok or lost_ok:
                self.obs_state = "clear"
                self.obs_first_stop_t = None
                self.obs_edge_since = 0.0
                rospy.loginfo("[DYN-OBS] 장애물 해제 → 주행 재개")

            # # 디버그 뷰만 갱신 후 조기 반환
            # bird_view_img = self.bird_view_transform(msg) * 255
            # cv2.imshow("bird_view_img", bird_view_img)
            # cv2.waitKey(1)
            return

        # 정지선이 검출 즉시
        if self.cross_flag and not prev_cross_flag:  # rising edge 감지
            self.stop_line_detected_time = current_time
            current_action = self.action_sequence[self.current_seq_index]

            # 4번째 정지선 검출 시 시간 기록
            if self.current_seq_index == 3:  # 4번째 정지선 (인덱스 3)
                self.fourth_stopline_passed = True
                self.fourth_stopline_time = current_time
                print("4번째 정지선 검출! 10초간 속도 500으로 고정")

            # 0.5초 카운트 시작
            self.post_stopline_straight = True
            self.post_stopline_start_time = current_time
            self.pending_action = current_action

            # straight 액션일 때 시작 시간 기록
            if current_action == "straight":
                self.straight_start_time = current_time

            # 다음 시퀀스로 이동
            self.current_seq_index = (self.current_seq_index + 1) % len(self.action_sequence)
            print(f"정지선 감지, 다음 동작: {current_action}")

        # 0.5초간 직진 유지
        if self.post_stopline_straight:
            if current_time - self.post_stopline_start_time <= 0.6:
                steer = 0.55
                speed = 1000.0
                print("정지선 후 0.5초 직진 유지")
            else:
                # 0.5초 경과, 실제 동작 시작
                self.post_stopline_straight = False
                print(f"동작 시작: {self.pending_action}")

        # action_sequence에 따른 처리
        elif self.pending_action == "straight":
            # 3초간 one_lane 모드, 이후 평상시로 복귀
            if current_time - self.straight_start_time <= 3.0:
                current_one_lane = self.one_lane_sequence[self.one_lane_index]
                center_index = self.get_center_index(msg, direction=current_one_lane)
                steer, speed = self.straight(center_index)
                print(f"straight (one_lane): {current_one_lane}")
            else:
                # 3초 경과, 다음 one_lane으로 이동 후 평상시로 복귀
                self.one_lane_index = (self.one_lane_index + 1) % len(self.one_lane_sequence)
                self.pending_action = None
                print(f"straight 종료, one_lane 다음: {self.one_lane_sequence[self.one_lane_index]}, 평상시로 복귀")

        elif self.pending_action == "left":
            steer, speed = self.turn(msg, direction="left")
            print(f"turn: left, steer={steer:.2f}, speed={speed:.0f}")
            # turn 함수에서 self.turning이 False가 되면 동작 종료
            if not self.turning:
                self.pending_action = None
                print("좌회전 완료, 평상시로 복귀")

        elif self.pending_action == "right":
            steer, speed = self.turn(msg, direction="right")
            print(f"turn: right, steer={steer:.2f}, speed={speed:.0f}")
            # turn 함수에서 self.turning이 False가 되면 동작 종료
            if not self.turning:
                self.pending_action = None
                print("우회전 완료, 평상시로 복귀")

        # 평상시 주행
        else:
            center_index = self.get_center_index(msg)
            steer, speed = self.straight(center_index)

        # 4번째 정지선 후 10초간 속도 500 고정
        if self.fourth_stopline_passed:
            elapsed_time = current_time - self.fourth_stopline_time
            if elapsed_time <= 6.0:
                # 0~6초: 원래 속도 유지 (speed 값 그대로)
                print(f"4번째 정지선 후 원래 속도 유지: {4.0 - elapsed_time:.1f}초 남음")
            elif elapsed_time <= 14.0:
                # 6~14초: turn_speed2(500) 속도
                speed = self.turn_speed2
                print(f"4번째 정지선 후 속도 500 고정: {14.0 - elapsed_time:.1f}초 남음")
            else:
                # 14초 이후: 원래 속도로 복귀하고 플래그 해제
                self.fourth_stopline_passed = False
                speed = self.speed
                print("14초 경과, 정상 속도로 복귀")

        # 안전 체크: steer 값 범위 제한
        steer = max(0.0, min(1.0, steer))

        # 안전 체크: speed 값 범위 제한
        speed = max(0.0, min(2000.0, speed))

        # 메시지 발행
        self.steer_msg.data = float(steer)
        self.speed_msg.data = float(speed)
        self.steer_pub.publish(self.steer_msg)
        self.speed_pub.publish(self.speed_msg)

        # # 디버그 뷰
        # bird_view_img = self.bird_view_transform(msg)*255
        # cv2.imshow("bird_view_img", bird_view_img)
        # cv2.waitKey(1)


def main():
    try:
        turtle_sub = Lane_sub()
        rospy.spin()
    except rospy.ROSInterruptException:  # ctrl C -> 강제종료
        pass


if __name__ == "__main__":
    main()
    # cv2.waitKey(1)
