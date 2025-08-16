from std_msgs.msg import Float64
from cv_bridge import CvBridge
from morai_msgs.msg import GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage
from collections import deque
import time
import cv2
import numpy as np
import rospy
2  # !/usr/bin/env python3
# -*- coding:utf-8 -*-


class LaneFollower:
    def __init__(self):
        rospy.init_node("LaneFollower_with_TrafficLight_Stopline")

        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback,
                         queue_size=1, buff_size=2**22, tcp_nodelay=True)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.tl_callback, queue_size=10)

        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)

        self.bridge = CvBridge()
        self.cv_img = None

        # === 신호등 관련 ===
        self.signal = None
        self.signal_is_red = False
        self.signal_is_green = False
        self.signal_is_left = False
        self.target_index = None
        self.last_tl_time = 0.0

        # === 정지선/래치 ===
        self.cross_flag = False
        self.stop_latched = False
        self.green_streak = 0
        self.GREEN_STREAK_N = 3

        # === 좌회전 래치 ===
        self.left_latched = False
        self.left_start_time = 0.0
        self.LEFT_STREAK_N = 2
        self.left_streak = 0

        self.TURN_MODE = 'lane'
        self.LEFT_TURN_TIME = 2.2
        self.LEFT_TURN_STEER = 0.22
        self.LEFT_TURN_SPEED = 700
        self.LEFT_TURN_BIAS = 0.07

        # === 조향 스무딩 ===
        self.STEER_EMA_ALPHA = 0.25
        self.STEER_SLEW_MAX = 0.08
        self.STEER_DEADZONE = 0.02
        self._steer_ema = 0.5
        self._steer_prev = 0.5

        # 중앙 안정화
        self.center_hist = deque(maxlen=5)

        # === PID (기본 게인) ===
        self.KP = 1.35        # 1.25 → 1.35 (급격한 코너 대응력 향상)
        self.KI = 0.00
        self.KD_BASE = 0.18   # 0.15 → 0.18 (안정성 향상)
        self.ERR_LP_ALPHA = 0.35  # 0.30 → 0.35 (빠른 반응)
        self.D_LP_ALPHA = 0.30    # 0.25 → 0.30 (미분 민감도 향상)
        self.I_CLAMP = 0.20

        # === 직진만 살살: 소프트존 & 스케줄 파라미터 ===
        self.E_SOFT = 0.06      # 0.07 → 0.06 (커브 감지 더 민감하게)
        self.A_MIN, self.A_MAX = 0.38, 1.00   # 0.42 → 0.38 (작은 에러도 더 크게 반응)
        self.G_MIN, self.G_MAX = 0.85, 0.7    # 0.9,0.6 → 0.85,0.7 (직진 약간 강화, 커브 더 강화)
        self.KD_NEAR, self.KD_FAR = 0.25, 0.10  # 0.20,0.13 → 0.25,0.10 (급격한 커브에서 더 민첩)

        # 커브 가중(출력 boost) – 급격한 코너 대응 강화
        self.CURVE_BOOST_START = 0.07   # 0.09 → 0.07 (더 빨리 커브로 인식)
        self.CURVE_BOOST_END = 0.20     # 0.22 → 0.20 (급격한 커브 임계값 낮춤)
        self.CURVE_BOOST_MAX = 0.05     # 0.02 → 0.05 (최대 부스트 2.5배 증가)

        # 추가: 극한 코너용 추가 부스트
        self.EXTREME_CURVE_START = 0.20  # 극한 코너 시작점
        self.EXTREME_CURVE_BOOST = 0.08  # 극한 코너 추가 부스트

        # 내부 PID 상태
        self.PID_OUTPUT_GAIN = 1.00
        self._e_curr = 0.0
        self._e_lp = 0.0
        self._e_lp_prev = 0.0
        self._d_lp = 0.0
        self._i_term = 0.0
        self._last_t = time.time()

    # ---------------------- 신호등 ----------------------
    def tl_callback(self, msg: GetTrafficLightStatus):
        if self.target_index is None:
            self.target_index = msg.trafficLightIndex
        if msg.trafficLightIndex != self.target_index:
            return
        raw = int(msg.trafficLightStatus)
        self.signal = raw
        self.signal_is_red = (raw & 0b00000001) != 0
        self.signal_is_green = (raw & 0b00010000) != 0
        self.signal_is_left = (raw & 0b00100000) != 0
        self.last_tl_time = time.time()

    # ---------------------- 이미지 ----------------------
    def img_callback(self, msg: CompressedImage):
        self.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.lane_follow()

    # ---------------------- 메인 ----------------------
    def lane_follow(self):
        if self.cv_img is None:
            return

        img = self.cv_img
        y, x = img.shape[:2]

        # ===== HSV 마스크 =====
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        yellow_range = cv2.inRange(img_hsv, np.array([10, 100, 100]), np.array([40, 255, 255]))
        white_range = cv2.inRange(img_hsv, np.array([0, 0, 150]),   np.array([179, 60, 255]))
        hsv_mask = cv2.bitwise_or(yellow_range, white_range)

        # ===== BEV =====
        src_points = np.array([[0, y], [283, 265], [x-283, 265], [x, y]], np.float32)
        dst_points = np.array([[x//4, y], [x//4, 0], [x//4*3, 0], [x//4*3, y]], np.float32)
        M = cv2.getPerspectiveTransform(src_points, dst_points)

        # ===== 정지선 =====
        bev_stop = cv2.warpPerspective(cv2.bitwise_and(img, img, mask=hsv_mask), M, (x, y))
        gray_stop = cv2.cvtColor(bev_stop, cv2.COLOR_RGB2GRAY)
        bin_stop = np.zeros_like(gray_stop, np.uint8)
        bin_stop[gray_stop > 40] = 1
        histogram_y = np.sum(bin_stop, axis=1)
        down_hist = histogram_y[y//2:]
        cross_indices = np.where(down_hist > 300)[0]
        self.cross_flag = (len(cross_indices) > 0 and (cross_indices[-1]-cross_indices[0]) > 40)

        # ===== 차선(점선 보강) =====
        v = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, gray_mask = cv2.threshold(v, 160, 255, cv2.THRESH_BINARY)
        lane_mask = cv2.bitwise_or(hsv_mask, gray_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel, 1)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN,  kernel, 1)

        bev_lane = cv2.warpPerspective(cv2.bitwise_and(img, img, mask=lane_mask), M, (x, y))
        gray_lane = cv2.cvtColor(bev_lane, cv2.COLOR_RGB2GRAY)
        bin_lane = np.zeros_like(gray_lane, np.uint8)
        bin_lane[gray_lane > 40] = 1

        # ===== BEV 히스토그램 중앙 =====
        histogram_x = np.sum(bin_lane, axis=0)
        lane_indices = np.where(histogram_x > 20)[0]

        if len(lane_indices) > 0:
            center_index = (lane_indices[0] + lane_indices[-1]) // 2

            # 중앙값 히스토리 안정화
            self.center_hist.append(center_index)
            center_smooth = int(np.median(self.center_hist))

            # e_norm = error / x  (≈ ±0.5 범위)
            error = (center_smooth - x//2)
            e_norm = error / float(x)

            # 데드존
            if abs(e_norm) < self.STEER_DEADZONE:
                e_norm = 0.0

            # ---------- 직진 얌전 + 커브 강화 스케줄 ----------
            a = abs(e_norm)

            # (A) 소프트존 스무스스텝 (0~E_SOFT 구간)
            s = a / max(1e-6, self.E_SOFT)
            s = float(np.clip(s, 0.0, 1.0))
            s = s*s*(3 - 2*s)  # smoothstep

            # (1) 에러 축소 (작을수록 A_MIN, 클수록 A_MAX)
            A = self.A_MIN + (self.A_MAX - self.A_MIN) * s
            e_shaped = e_norm * A

            # (2) 기본 출력 스케일 (작을수록 G_MIN, 클수록 G_MAX)
            g_base = self.G_MIN + (self.G_MAX - self.G_MIN) * s

            # (3) 커브 부스트: |e|가 CURVE_BOOST_START~END 사이면 g에 +0~MAX
            if a <= self.CURVE_BOOST_START:
                boost = 0.0
            elif a >= self.CURVE_BOOST_END:
                boost = self.CURVE_BOOST_MAX
                # 극한 코너 추가 부스트
                if a >= self.EXTREME_CURVE_START:
                    boost += self.EXTREME_CURVE_BOOST
            else:
                t = (a - self.CURVE_BOOST_START) / (self.CURVE_BOOST_END - self.CURVE_BOOST_START)
                t = np.clip(t, 0.0, 1.0)
                t = t*t*(3 - 2*t)  # smoothstep
                boost = self.CURVE_BOOST_MAX * t

            self.gain_sched = g_base + boost

            # (4) KD 스케줄: 직진(작은 a)일수록 KD 큼(댐핑↑), 급격한 커브일수록 KD 작음(민첩↑)
            self.kd_sched = self.KD_NEAR + (self.KD_FAR - self.KD_NEAR) * s

            # 극한 코너에서 KD 추가 감소 (더 민첩한 반응)
            if a >= self.EXTREME_CURVE_START:
                extreme_factor = min((a - self.EXTREME_CURVE_START) / 0.1, 1.0)
                self.kd_sched *= (1.0 - 0.3 * extreme_factor)  # KD를 최대 30% 감소
            # --------------------------------------------------

            # PID 입력 업데이트
            self._e_curr = float(e_shaped)
            steer_from_pid = self.pid_step()  # 0~1
            steer_lane = steer_from_pid
        else:
            steer_lane = 0.5

        # ===== 신호 recent 여부 =====
        tl_recent = (time.time() - self.last_tl_time) < 1.0

        # ===== 좌회전 래치 =====
        if self.signal_is_left and self.cross_flag and tl_recent:
            self.left_streak = min(self.left_streak + 1, 1000)
        else:
            self.left_streak = 0

        if (not self.left_latched) and (self.left_streak >= self.LEFT_STREAK_N):
            self.left_latched = True
            self.left_start_time = time.time()
            self.stop_latched = False
            self.green_streak = 0

        if self.left_latched and (time.time() - self.left_start_time) >= self.LEFT_TURN_TIME and self.TURN_MODE == 'fixed':
            self.left_latched = False

        # ===== 정지 래치 =====
        red_and_stop = (self.signal_is_red and self.cross_flag and tl_recent)
        if red_and_stop and not self.left_latched:
            self.stop_latched = True
            self.green_streak = 0
        elif self.stop_latched:
            if self.signal_is_green and tl_recent:
                self.green_streak += 1
                if self.green_streak >= self.GREEN_STREAK_N:
                    self.stop_latched = False
                    self.green_streak = 0
            else:
                self.green_streak = 0

        # ===== 조향/속도 =====
        if self.left_latched:
            if self.TURN_MODE == 'fixed':
                steer_target = float(np.clip(self.LEFT_TURN_STEER, 0.0, 1.0))
            else:
                steer_target = float(np.clip(steer_lane - self.LEFT_TURN_BIAS, 0.0, 1.0))
            speed_value = float(self.LEFT_TURN_SPEED)
        else:
            steer_target = float(np.clip(steer_lane, 0.0, 1.0))
            speed_value = 0.0 if self.stop_latched else 900.0

        # --- EMA + Slew ---
        self._steer_ema = (1 - self.STEER_EMA_ALPHA)*self._steer_ema + self.STEER_EMA_ALPHA*steer_target
        delta = np.clip(self._steer_ema - self._steer_prev, -self.STEER_SLEW_MAX, self.STEER_SLEW_MAX)
        steer_sm = self._steer_prev + delta
        self._steer_prev = steer_sm

        if self.stop_latched:
            steer_sm = 0.5

        steer_value_pub = float(np.clip(steer_sm, 0.0, 1.0))

        # 퍼블리시
        self.steer_pub.publish(Float64(steer_value_pub))
        self.speed_pub.publish(Float64(speed_value))

        # 디버그
        print(f"e:{self._e_curr:.3f} g:{getattr(self,'gain_sched',1.0):.2f} kd:{getattr(self,'kd_sched',self.KD_BASE):.2f} "
              f"steer:{steer_value_pub:.3f} stop:{self.stop_latched} left:{self.left_latched} speed:{speed_value:.0f}")

    # ---------------------- PID (스케줄 반영) ----------------------
    def pid_step(self):
        now = time.time()
        dt = max(1e-3, now - self._last_t)
        self._last_t = now

        # LPF
        self._e_lp = (1.0 - self.ERR_LP_ALPHA) * self._e_lp + self.ERR_LP_ALPHA * self._e_curr
        d_raw = (self._e_lp - self._e_lp_prev) / dt
        self._d_lp = (1.0 - self.D_LP_ALPHA) * self._d_lp + self.D_LP_ALPHA * d_raw
        self._e_lp_prev = self._e_lp

        kd = getattr(self, 'kd_sched', self.KD_BASE)
        p_term = self.KP * self._e_lp
        d_term = kd * self._d_lp

        i_candidate = self._i_term + self.KI * self._e_lp * dt
        i_candidate = float(np.clip(i_candidate, -self.I_CLAMP, self.I_CLAMP))

        pid_sum = p_term + i_candidate + d_term

        g = getattr(self, 'gain_sched', self.PID_OUTPUT_GAIN)
        steer = 0.5 + g * pid_sum
        steer = float(np.clip(steer, 0.0, 1.0))

        if 0.01 < steer < 0.99:
            self._i_term = i_candidate

        return steer


if __name__ == "__main__":
    try:
        LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
