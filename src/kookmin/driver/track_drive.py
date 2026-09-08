#!/usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================
# 본 프로그램은 2025 제8회 국민대 자율주행 경진대회에서
# 예선과제를 수행하기 위한 파일입니다.
# 예선과제 수행 용도로만 사용가능하며 외부유출은 금지됩니다.
# =============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
# =============================================
import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from collections import deque

from sklearn.cluster import KMeans

# =============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
# =============================================
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
ranges = None  # 라이다 데이터를 담을 변수
motor = None  # 모터노드
motor_msg = XycarMotor()  # 모터 토픽 메시지
Fix_Speed = 10  # 모터 속도 고정 상수값
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지
Max_Speed = 10  # 최대 속도
Min_Speed = 2  # 최소 속도
Base_Speed = 5  # 기본 속도
avoid_state = 'init' #회피 상태태

left_pts_history = deque(maxlen=10)  # 히스토리 길이 좀 늘림
right_pts_history = deque(maxlen=10)
lane_center_history = deque(maxlen=10)  # 차선 중앙 이동평균용
last_angle = 0
lane_mask = None
last_known_left_lines = []
last_known_right_lines = []
mycar_speed = 30  # 내 차량의 속도

g_cone_path_is_valid = False # follow_cone_lane 성공 여부 플래그
g_cone_lost_timestamp = None   # 라바콘 경로 유실 시작 시간
CONE_AVOID_GRACE_PERIOD = 1.0  # 초 단위 유예 시간

# =============================================
# 라이다 스캔정보로 그림을 그리기 위한 변수
# =============================================
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-120, 120)
ax.set_ylim(-120, 120)
ax.set_aspect('equal')
lidar_points, = ax.plot([], [], 'bo')


# =============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# =============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")


# =============================================
# 콜백함수 - 라이다 토픽을 받아서 처리하는 콜백함수
# =============================================
def lidar_callback(data):
    global ranges
    ranges = data.ranges[0:360]


# =============================================
# 모터로 토픽을 발행하는 함수
# =============================================
def drive(angle, speed):
    print(f"[DEBUG] drive 함수 호출됨 - angle: {angle}, speed: {speed}")
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)    

# =============================================
# 차량과 라바콘을 카메라 영상으로 구분하는 함수
# =============================================
def detect_object():
    global image
    if image is None:
        return 'cone'
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_orange = np.array([5, 100, 100])
    upper_orange = np.array([20, 255, 255])
    orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

    orange_pixels = cv2.countNonZero(orange_mask)

    if orange_pixels > 100:
        return 'cone'
    else:
        return 'vehicle'


# =============================================
# 라이다 센서로 전방 장애물 존재 여부를 판단하는 함수
# =============================================
def is_obstacle_close_by_lidar():
    global ranges
    if ranges is None:
        return False

    # 전방 20도 범위 데이터 추출
    front = ranges[340:] + ranges[:20]

    # 유효한 거리값만 필터링
    valid = [d for d in front if 0.01 < d < float('inf')]

    # 일정 거리 이하 물체 존재 여부 반환
    return min(valid) < 8 if valid else False

# =============================================
# 현재 차량이 우측 차선에 있는지 판단하는 함수
# =============================================
def is_on_right_lane():
    global image
    if image is None:
        return True

    # 관심 영역 설정 및 HSV 변환
    height, width, _ = image.shape
    roi = image[int(height * 0.6):, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 노란 점선, 흰 실선 추출
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 50, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    # 각 라인의 중심 좌표 비교
    yellow_pos = cv2.findNonZero(yellow_mask)
    white_pos = cv2.findNonZero(white_mask)

    if yellow_pos is None or white_pos is None:
        return True

    yellow_mean = np.mean(yellow_pos[:, 0, 0])
    white_mean = np.mean(white_pos[:, 0, 0])
    return yellow_mean < white_mean


# =============================================
# 여기서부터 is_path_clear까지 차량 회피코드임 (디버그를 위해 주석바리우스를이제 어떻게 돌아가는지에 대해 설명을 하겠읍니다하겠읍니다)
# 전방 최소거리 check  (vehicle_avoid_mode에 들어가는거는 20m 남았을때지만(is_obstacle_close_by_lida), 
# 20m는 여유가 매우 있는 상태임 그래서 이 기준으로 계속 상대 속도 check를 하면서 회피하도록 설계를 했습니다.)
# =============================================
from collections import deque
front_min_buffer = deque(maxlen=5)  # 프레임 누적용 버퍼 recovery 상태로 사인함수로 돌아가면서 recovery에 들어가는데, recovery를 하면서 전방이 아닌 옆자선에 있는 차를
                                     # 앞에 차가 있다고 인식해 버그가 생겨 이를 막기 위해 10프레임의 평균값으로 판단을 하는 코드를 짜봤읍니다.

def get_front_min_distance():
    global front_min_buffer, ranges
    if ranges is None:
        return float('inf')

    # 전방 각도 범위 추출 (각도는 전방 20도도)
    front = ranges[350:] + ranges[:10]
    valid = [d for d in front if 0.01 < d < float('inf')]
    current_min = min(valid) if valid else float('inf')

    # 프레임 누적 버퍼에 추가
    front_min_buffer.append(current_min)

    # 최근 5프레임 평균 반환
    smoothed_min = sum(front_min_buffer) / len(front_min_buffer) if front_min_buffer else current_min
    return smoothed_min



# =========================================
# === init 에서 plan으로 넘어가기 (2중 안전장치라 생각하쇼) ===#

avoid_init_start_time = None

def handle_avoid_init():
    global avoid_state, avoid_init_start_time

    current_min = get_front_min_distance()
    print(f"[INIT] 전방 거리: {current_min:.2f}m")

    # 처음 진입한 순간 시간 저장
    if avoid_init_start_time is None:
        avoid_init_start_time = time.time()

    # 일정 시간 동안은 차선 따라가기 (ex: 약간 수평될 때까지)
    elapsed = time.time() - avoid_init_start_time
    if elapsed < 2.5:
        final_mask, left_lines, right_lines, center = get_line_mask()
        follow_lane(left_lines, right_lines, center)
        print(f"[INIT] 🔄 회피 전 준비 중... {elapsed:.2f}s 경과")
        return

    if current_min < 13:  # 기존 진입 조건 유지
        print("[INIT] ✅ 회피 FSM 진입 조건 만족 → 상태 전이: PLAN")
        avoid_state = "plan"
        avoid_init_start_time = None  # 초기화
    else:
        print("[INIT] 🚫 거리 충분함 → 회피 FSM 진입 보류")

# =============================================
# 이제 plan에서 저번에 설정했던 기준보다 더 간소화해 자기가 어디 차선에 있는지 check
# =============================================
from collections import deque
prev_min = float('inf')
delta_buffer = deque(maxlen=5)
vehicle_avoid_start_time=0
MAX_STEER=25
def handle_avoid_plan():
    global avoid_state, initial_steer, vehicle_avoid_start_time

    current_min = get_front_min_distance()
    print(f"[PLAN] 📏 전방 거리: {current_min:.2f}m")

    if current_min > 20:
        maintain_distance()

    elif current_min < 15:
        # 차선 위치에 따라 회피 방향 결정
        direction = 'left' if is_on_right_lane() else 'right'
        initial_steer = -MAX_STEER if direction == 'left' else MAX_STEER

        # 회피 시작 시간 기록
        vehicle_avoid_start_time = time.time()
        avoid_state = "avoiding"

        print(f"[PLAN] ✅ 회피 방향: {direction.upper()} | steer: {initial_steer}")
        print("[PLAN] 🔄 상태 전이: 'avoiding'")
        handle_avoid_running()
    else:
        print("[PLAN] ⏸ 거리 충분 → 회피 진입 보류 (상태 유지)")
        # 상태 그대로 유지 (plan 반복)

# =============================================
# 회피기동 실행
# =============================================
def handle_avoid_running():
    global avoid_state, recovery_start_time,t
    global prev_min, delta_buffer
    
    elapsed = time.time() - vehicle_avoid_start_time
    duration = 1.2  # 회피 조향에 걸리는 시간 (초) → 조정 가능
    t = min(elapsed / duration, 1.0)  # 0~1로 정규화

    steer = initial_steer * math.sin(math.pi * t/2)
    print("dr1")
    drive(angle=steer, speed=60)

    print(f"[AVOIDING] 🕒 경과: {elapsed:.2f}s | steer: {steer:.2f}")

    if elapsed >= duration:
        print("[AVOIDING] ✅ 회피 완료 → 상태 전이: 'recovery'")
        recovery_start_time = time.time()
        avoid_state = 'recovery'
        vehicle_recovery()

#=============================================
# === 회피 후에 각도를 복원하도록 추가 ===
#=============================================
recovery_start_time = 0
def vehicle_recovery():
    global avoid_state, recovery_start_time,recovery_end_time

    elapsed = time.time() - recovery_start_time
    duration = 1.2
    t = min(elapsed / duration, 1.0)

    # 단순한 복귀 조향만 수행
    steer = - initial_steer * (1 - math.cos(math.pi * t / 2))
    steer = np.clip(steer, -35, 35)

    print("dr2")
    drive(angle=steer, speed=45)
    print(f"[RECOVERY] ⏱ {elapsed:.2f}s | steer: {steer:.2f}")


    if elapsed >= duration:
        recovery_end_time = time.time()
        avoid_state = 'post_recovery'

#===========================================
# === recovery 안정성을 위해서 recovery후에 1초간 lane_follow 하게 설정을 하고 다음 모드로 넘어가게했읍니다 ===#
# === 거기에 obstacle true가 아닐시에 lane follow로 모드 자체를 탈출하도록 설정해서 이 루프읜 안정성을 높였읍니다 흐흐 ===#
recovery_end_time = 0  # post_recovery 시작 시각

# === 차선 주행용 전역 차선 정보 ===
left_lines = []
right_lines = []
center = 0

def handle_post_recovery():
    global avoid_state, mode, mycar_speed
    final_mask, left_lines, right_lines, center = get_line_mask()  
    elapsed = time.time() - recovery_end_time
    print(f"[POST_RECOVERY] ⏱ 경과 시간: {elapsed:.2f}s")

    # 차선 디버깅 정보 출력
    print(f"[POST_RECOVERY] 현재 차선 정보:")
    print(f"  - left_lines: {left_lines if left_lines else '없음'}")
    print(f"  - right_lines: {right_lines if right_lines else '없음'}")
    print(f"  - center 기준: {center}")

    if elapsed < 4.5:
        print("[POST_RECOVERY] 🛣 4.5초 미만 → follow_lane 실행 중")
        follow_lane(left_lines, right_lines, center)
        if last_angle < 10 and left_lines and right_lines:
            mycar_speed = 80
            print("dr14")
            drive(last_angle, mycar_speed)

    elif left_lines and right_lines:
        current_min = get_front_min_distance()
        print(f"[POST_RECOVERY] 📏 전방 최소 거리: {current_min:.2f}m")

        if current_min < 50.0:
            print("[POST_RECOVERY] ⚠️ 전방에 차량 있음 → maintain_distance로 전이")
            avoid_state = 'maintain_distance'
        else:
            print("[POST_RECOVERY] ✅ 전방 클리어 → mode: lane_follow로 전이")
            avoid_state = 'init'
            mode = 'lane_follow'
    
    else:
        print("dr6")
        drive(last_angle,20)


# =============================================
# 밑에 루프를 좀 쉽게 하기위해서 인식->회피->복구를 한 함수로 설정함함
# =============================================
def handle_full_avoid():
    global avoid_state

    if avoid_state == "plan":
        handle_avoid_plan()
    elif avoid_state == "avoiding":
        handle_avoid_running()
    elif avoid_state == "recovery":
        vehicle_recovery()

#========================================
#추월하는 함수 설정하기엔 노이즈가 많아서 차가 회피를 하고 steer값 복구를하고 차피 다른 차가 빠르니까
# 회피 했을 때 앞의 차와의 거리를 유지하도록 speed 설정을 한거임
# 그리고 앞에차가 멈췄다고 판단이 들면 회피 하도록 하게 설정함

from collections import deque

distance_buffer = deque(maxlen=6)  # 6개 프레임으로 변경 (현재 + 이전 5)

TARGET_DISTANCE = 13.0  # 목표 거리 10m
plan_trigger_distance = 17  # 일정 거리 이상 벌어지면 plan 상태로 전이

def maintain_distance():
    global avoid_state, distance_buffer, mycar_speed

    current_min = get_front_min_distance()
    distance_buffer.append(current_min)

    # Δ거리 계산: 최근 프레임 거리 변화 평균
    if len(distance_buffer) < 2:
        avg_delta = 0
    else:
        deltas = [distance_buffer[i] - distance_buffer[i-1] for i in range(1, len(distance_buffer))]
        avg_delta = sum(deltas) / len(deltas)

    print(f"[DISTANCE] 📏 거리: {current_min:.2f}m | Δ평균: {avg_delta:.3f}")

    base_speed = 30
    k = 100

    distance_error = current_min - TARGET_DISTANCE
    
    target_speed = base_speed + k * distance_error - k * avg_delta
    mycar_speed =  max(55, min(75, target_speed))

    print(f"[DISTANCE] 🧭 속도 조절 → target_speed: {target_speed:.2f}, mycar_speed: {mycar_speed:.2f}")
    print("dr3")
    drive(last_angle, mycar_speed)

    # 거리 trigger로 판단하는 걸로 바꾸었습니다.
    if current_min < plan_trigger_distance:
        avoid_state = 'plan'
        distance_buffer.clear()
        handle_avoid_plan()
        
#================================================
# 위에 설정한 함수를 그냥 호출되면 fucking fast track으로 되게 설정했읍니다.
#########################################################
def handle_vehicle_avoidance():
    global avoid_state

    if avoid_state == 'init':
        handle_avoid_init()
        return

    if avoid_state == 'plan':
        handle_avoid_plan()
        return

    if avoid_state == 'avoiding':
        handle_avoid_running()
        return

    if avoid_state == 'recovery':
        vehicle_recovery()
        return

    if avoid_state == 'maintain_distance':
        maintain_distance()
        return

    if avoid_state == 'post_recovery':
        handle_post_recovery()
        return

    print(f"[VEHICLE_AVOID] 알 수 없는 상태: {avoid_state} → init으로 초기화")
    avoid_state = 'init'


# =======================================================
# === 전역 변수 선언 (파일 상단에 위치해야 합니다) ===
# =======================================================
last_known_left_x = None
last_known_right_x = None

# =======================================================
# === 전역 변수 선언 (파일 상단에 위치해야 합니다) ===
# =======================================================
last_known_left_x = None
last_known_right_x = None

# =======================================================
# === 헬퍼 함수 (경로 보간 전 데이터 정리용) ===
# =======================================================
def clean_path_for_interp(path):
    if path is None or len(path) < 2: return path
    # path.copy()를 path로 변경
    cleaned_path = [path[0]]
    for i in range(1, len(path)):
        if path[i][1] != cleaned_path[-1][1]:
            cleaned_path.append(path[i])
    return cleaned_path

# =======================================================
# === 메인 라바콘 주행 함수 (요청사항 반영) ===
# =======================================================
def follow_cone_lane():
    global image, last_angle
    global g_cone_path_is_valid # 전역 플래그 사용 선언
    
    path_formation_method = "Initial" # 지역 변수로 사용
    g_cone_path_is_valid = False # 함수 시작 시 항상 False로 초기화

    if image is None or image.size == 0:
        return

    # --- 1. 이미지 처리 및 통합된 라바콘 포인트 검출 ---
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5, 80, 80]); upper_orange = np.array([25, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    height, width = mask.shape
    
    actual_roi_start_y = int(height * 0.6)
    roi_cv = mask[actual_roi_start_y:, :]

    blurred = cv2.GaussianBlur(roi_cv, (5, 5), 0)
    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    MIN_AREA = 50
    raw_detected_points_in_roi = []
    for cnt in contours:
        area = cv2.contourArea(cnt); M = cv2.moments(cnt)
        if area < MIN_AREA or M['m00'] == 0: continue
        cx = int(M['m10'] / M['m00']); cy = int(M['m01'] / M['m00'])
        raw_detected_points_in_roi.append((cx, cy))

    merged_cone_points_in_roi = []
    processed_indices = [False] * len(raw_detected_points_in_roi)
    MERGE_DISTANCE_THRESHOLD = 35
    for i in range(len(raw_detected_points_in_roi)):
        if processed_indices[i]: continue
        current_group = [raw_detected_points_in_roi[i]]
        processed_indices[i] = True
        for j in range(i + 1, len(raw_detected_points_in_roi)):
            if processed_indices[j]: continue
            pt1 = raw_detected_points_in_roi[i]; pt2 = raw_detected_points_in_roi[j]
            dist = math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
            if dist < MERGE_DISTANCE_THRESHOLD:
                current_group.append(pt2); processed_indices[j] = True
        if current_group:
            sum_x = sum(p[0] for p in current_group); sum_y = sum(p[1] for p in current_group)
            merged_cone_points_in_roi.append((int(sum_x / len(current_group)), int(sum_y / len(current_group))))

    vis = cv2.cvtColor(roi_cv, cv2.COLOR_GRAY2BGR)
    screen_center_x_roi = vis.shape[1] // 2
    roi_actual_height = vis.shape[0]

    # --- 2. ROI 영역별 라바콘 분리 및 처리 ---
    y_ignore_top_abs = roi_actual_height * 0.15
    y_kmeans_top_abs = roi_actual_height * 0.30
    cones_for_kmeans = []
    cones_for_driving_path_classification = []

    for pt_roi in merged_cone_points_in_roi:
        if pt_roi[1] <= y_ignore_top_abs: continue
        elif pt_roi[1] <= y_kmeans_top_abs: cones_for_kmeans.append(pt_roi)
        else: cones_for_driving_path_classification.append(pt_roi)

    k_means_left_far = []
    k_means_right_far = []
    if len(cones_for_kmeans) >= 2:
        points_k_np = np.array(cones_for_kmeans)
        kmeans = KMeans(n_clusters=2, init='k-means++', n_init=10, random_state=0)
        try:
            labels = kmeans.fit_predict(points_k_np)
            centroids = kmeans.cluster_centers_
            if centroids[0][0] < centroids[1][0]: left_label_k, right_label_k = 0, 1
            else: left_label_k, right_label_k = 1, 0
            k_means_left_far = [tuple(p) for p in points_k_np[labels == left_label_k]]
            k_means_right_far = [tuple(p) for p in points_k_np[labels == right_label_k]]
        except ValueError as e:
            print(f"[KMEANS_FAR] Error: {e}.")
            pass 
            
    near_classified_left = []
    near_classified_right = []
    anchor_center_x_roi = screen_center_x_roi 
    y_near_classify_split_roi = roi_actual_height * 0.5
    STEERING_INFLUENCE = 1.5
    for cx_roi, cy_roi in cones_for_driving_path_classification:
        boundary_roi = screen_center_x_roi
        if cy_roi > y_near_classify_split_roi: boundary_roi = anchor_center_x_roi
        else: boundary_roi = screen_center_x_roi + (last_angle * STEERING_INFLUENCE)
        if cx_roi < boundary_roi: near_classified_left.append((cx_roi, cy_roi))
        else: near_classified_right.append((cx_roi, cy_roi))

    # --- 3. 최종 차선 후보군 통합 및 경로 생성 ---
    bottom_y_roi = roi_actual_height - 1
    fixed_left_anchor_x_roi = screen_center_x_roi - 270
    fixed_right_anchor_x_roi = screen_center_x_roi + 270
    left_anchor_roi = (fixed_left_anchor_x_roi, bottom_y_roi)
    right_anchor_roi = (fixed_right_anchor_x_roi, bottom_y_roi)
    current_left_pts = sorted([left_anchor_roi] + near_classified_left + k_means_left_far, key=lambda p: p[1], reverse=True)
    current_right_pts = sorted([right_anchor_roi] + near_classified_right + k_means_right_far, key=lambda p: p[1], reverse=True)
    current_left_pts = [current_left_pts[i] for i in range(len(current_left_pts)) if i == 0 or current_left_pts[i] != current_left_pts[i-1]]
    current_right_pts = [current_right_pts[i] for i in range(len(current_right_pts)) if i == 0 or current_right_pts[i] != current_right_pts[i-1]]

    MAX_CONES_PER_LANE = 7; MAX_NEXT_CONE_DISTANCE = 280; PATH_BUILD_SIDE_TOLERANCE_ROI = 180
    final_left_path = None
    if len(current_left_pts) >= 2:
        path = [current_left_pts[0]] 
        pool = list(current_left_pts[1:])
        for _ in range(MAX_CONES_PER_LANE -1):
            if not pool: break
            current_ref = path[-1]
            pool.sort(key=lambda p: math.sqrt((p[0]-current_ref[0])**2 + (p[1]-current_ref[1])**2)) 
            best_next = pool.pop(0)
            if math.sqrt((best_next[0]-current_ref[0])**2 + (best_next[1]-current_ref[1])**2) < MAX_NEXT_CONE_DISTANCE and best_next[1] < current_ref[1]:
                path.append(best_next)
            else: break
        final_left_path = path
        
    final_right_path = None
    if len(current_right_pts) >= 2:
        path = [current_right_pts[0]] 
        pool = list(current_right_pts[1:])
        actual_cones_in_left_path = [p for p in (final_left_path if final_left_path else []) if p != left_anchor_roi]
        pool = [p for p in pool if p not in actual_cones_in_left_path]
        for _ in range(MAX_CONES_PER_LANE -1):
            if not pool: break
            current_ref = path[-1]
            pool.sort(key=lambda p: math.sqrt((p[0]-current_ref[0])**2 + (p[1]-current_ref[1])**2))
            best_next = pool.pop(0)
            if math.sqrt((best_next[0]-current_ref[0])**2 + (best_next[1]-current_ref[1])**2) < MAX_NEXT_CONE_DISTANCE and best_next[1] < current_ref[1]:
                path.append(best_next)
            else: break
        final_right_path = path
        
    # --- 4. 경로 유효성 판단 및 가상 차선 생성 ---
    MIN_VALID_PATH_LEN = 3 
    LANE_WIDTH_ROI = 400      
    left_path_good = final_left_path and len(final_left_path) >= MIN_VALID_PATH_LEN
    right_path_good = final_right_path and len(final_right_path) >= MIN_VALID_PATH_LEN
    path_status_message = ""
    
    base_speed = 5
    reduced_speed_virtual = 3
    # [요청사항 반영] 콘 개수 부족 시 속도
    speed_when_cones_insufficient = 7
    
    base_steer_gain = 0.00966352
    aggressive_steer_gain_multiplier = 1.5
    current_speed = base_speed
    current_steer_gain = base_steer_gain

    if left_path_good and right_path_good:
        path_status_message = "Both OK"
        # current_speed, current_steer_gain은 기본값 사용
    elif right_path_good and not left_path_good: 
        final_left_path = [(x - LANE_WIDTH_ROI, y) for x, y in final_right_path]
        path_status_message = "Virtual L"
        current_speed = reduced_speed_virtual
        current_steer_gain = base_steer_gain * aggressive_steer_gain_multiplier
    elif left_path_good and not right_path_good: 
        final_right_path = [(x + LANE_WIDTH_ROI, y) for x, y in final_left_path]
        path_status_message = "Virtual R"
        current_speed = reduced_speed_virtual
        current_steer_gain = base_steer_gain * aggressive_steer_gain_multiplier
    else: # 양쪽 모두 유효하지 않음 (MIN_VALID_PATH_LEN 미만)
        path_status_message = "Both Paths Invalid - Using Last Angle"
        # [요청사항 반영] 속도를 10으로, 이전 각도 유지
        drive(last_angle, speed_when_cones_insufficient) 
        
        cv2.putText(vis, path_status_message, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(vis, f"Steer: {last_angle:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if 'left_anchor_roi' in locals(): cv2.circle(vis, left_anchor_roi, 8, (255,255,255), -1)
        if 'right_anchor_roi' in locals(): cv2.circle(vis, right_anchor_roi, 8, (255,255,255), -1)
        #cv2.imshow("Cone Lane Final", vis)
        g_cone_path_is_valid = False # 경로 생성 실패
        return 

    # --- 5. 중앙 경로 계산 및 조향각 제어 ---
    y_vals = np.linspace(0, roi_actual_height - 1, 50)
    cleaned_left_path = clean_path_for_interp(final_left_path)
    cleaned_right_path = clean_path_for_interp(final_right_path)

    if len(cleaned_left_path) < 2 or len(cleaned_right_path) < 2:
        # [요청사항 반영] 보간 실패 시에도 속도는 위에서 결정된 current_speed 또는 10
        # (이 경우는 보통 양쪽 경로가 good하지 않아 위에서 return되지만, 만약을 위해)
        final_speed = speed_when_cones_insufficient if path_status_message == "Both Paths Invalid - Using Last Angle" else current_speed
        drive(last_angle, final_speed) 
        cv2.putText(vis, "Interpolation Fail!", (20,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
        g_cone_path_is_valid = False # 경로 생성 실패
    else:
        left_x_coords = [p[0] for p in cleaned_left_path]; left_y_coords = [p[1] for p in cleaned_left_path]
        right_x_coords = [p[0] for p in cleaned_right_path]; right_y_coords = [p[1] for p in cleaned_right_path]
        interp_left_x = np.interp(y_vals, left_y_coords[::-1], left_x_coords[::-1])
        interp_right_x = np.interp(y_vals, right_y_coords[::-1], right_x_coords[::-1])
        mid_x_polyline = (interp_left_x + interp_right_x) / 2
        lane_center_roi = np.mean(mid_x_polyline)
        error = lane_center_roi - screen_center_x_roi 
        steer = current_steer_gain * error * 100 
        steer = np.clip(steer, -85, 85); max_delta = 15 
        steer = max(min(steer, last_angle + max_delta), last_angle - max_delta)
        drive(steer, current_speed) # 최종 결정된 속도와 조향각 사용
        last_angle = steer
        g_cone_path_is_valid = True # 성공적으로 주행 명령

    # --- 6. 시각화 ---
    # ... (이전 시각화 코드와 동일) ...
    # path_formation_method 변수는 이 버전에서 명시적으로 사용되지 않으므로 시각화에서 제거하거나 주석처리
    # cv2.putText(vis, f"Form: {path_formation_method}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1)
    if final_left_path:
        pts_left_visual = np.array(final_left_path, dtype=np.int32)
        if len(pts_left_visual) > 0 :
             cv2.polylines(vis, [pts_left_visual], isClosed=False, color=(0, 0, 255), thickness=3)
             for pt_idx, pt in enumerate(final_left_path):
                is_anchor = (pt == left_anchor_roi)
                color = (255,255,255) if is_anchor else (0,255,255)
                cv2.circle(vis, pt, 7, color, -1)
    if final_right_path:
        pts_right_visual = np.array(final_right_path, dtype=np.int32)
        if len(pts_right_visual) > 0:
            cv2.polylines(vis, [pts_right_visual], isClosed=False, color=(255, 0, 0), thickness=3)
            for pt_idx, pt in enumerate(final_right_path):
                is_anchor = (pt == right_anchor_roi)
                color = (255,255,255) if is_anchor else (0,0,255)
                cv2.circle(vis, pt, 7, color, -1)

    if 'labels' in locals() and 'points_k_np' in locals() and len(cones_for_kmeans) >=2:
        temp_colors_k = [(255,100,0), (0,100,255)] 
        if 'labels' in locals() and len(cones_for_kmeans) >= 2 and 'points_k_np' in locals(): # 'labels' 변수 존재 확인
            unique_k_labels = np.unique(labels)
            for i, label_id_k in enumerate(unique_k_labels):
                if i < len(temp_colors_k) and label_id_k != -1 : # 노이즈 레이블(-1) 제외 (KMeans는 보통 -1 안씀)
                    cluster_points_k = points_k_np[labels == label_id_k]
                    color_k = temp_colors_k[i]
                    for pt_np_k in cluster_points_k:
                        pt_k = tuple(pt_np_k.astype(int))
                        cv2.circle(vis, pt_k, 6, color_k, 2)
                        if 'centroids' in locals() and i < len(centroids):
                             cv2.circle(vis, (int(centroids[i][0]), int(centroids[i][1])), 3, color_k, -1)

    if 'mid_x_polyline' in locals() and mid_x_polyline is not None and ('lane_center_roi' in locals()):
        pts_mid = np.array(list(zip(mid_x_polyline.astype(np.int32), y_vals.astype(np.int32))))
        cv2.polylines(vis, [pts_mid], isClosed=False, color=(0, 255, 0), thickness=3)
        cv2.line(vis, (screen_center_x_roi, roi_actual_height), (int(lane_center_roi), int(np.mean(y_vals))), (255, 255, 0), 3)

    cv2.putText(vis, f"Steer: {last_angle:.1f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    # cv2.putText(vis, f"Form: {path_formation_method}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1) # 이 변수는 더 이상 업데이트되지 않음
    if path_status_message:
        cv2.putText(vis, path_status_message, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 150, 255), 2)
    #cv2.imshow("Cone Lane Final", vis)
    

# =============================================
# 빨간 신호등이 감지된 경우 정지 및 출발하는 함수 (수정됨)
# =============================================
def handle_traffic_light():
    global image, Fix_Speed

    print("[INFO] Traffic light sequence started.")
    
    # 신호등이 초록불이 될 때까지 무한정 대기
    while True:
        # 현재 신호등 상태를 받아옴
        light_state = detect_traffic_light()
        
        # 멈춰있는 동안 화면 갱신
        if image.size != 0:
            display_img = image.copy()
            height, _, _ = image.shape
            roi_y_end = int(height * 0.4)
            cv2.rectangle(display_img, (0, 0), (display_img.shape[1], roi_y_end), (0, 255, 255), 2)
            cv2.putText(display_img, f"Light: {light_state.upper()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            #cv2.imshow("original", display_img)
            cv2.waitKey(1)

        # 상태에 따라 행동 결정
        if light_state == 'green':
            print("[INFO] GREEN LIGHT detected. Proceeding...")
            break  # 초록불이면 대기 루프 탈출
        
        elif light_state == 'red':
            print("[INFO] RED light detected. Waiting...")
            drive(0, 0) # 정지
        
        else: # 'none'일 경우
            print("[INFO] No clear light signal. Waiting...")
            drive(0, 0) # 안전을 위해 정지

        time.sleep(0.3) # 0.3초마다 신호 확인

    # --- 대기 루프 탈출 후 (초록불을 본 후) ---
    print("[INFO] Driving straight for 1 second...")
    drive(0, 30)  # 조향각 0(직진), 설정된 속도로 주행
    time.sleep(1.0)      # 1초 동안 이 상태를 유지

    print("[INFO] Traffic light sequence finished.")


# =============================================
# 신호등의 상태를 판단하는 함수 (수정된 버전)
# =============================================
def detect_traffic_light():
    global image
    if image.size == 0:
        return "none" # 이미지 없을 때도 문자열 "none" 반환

    height, width, _ = image.shape
    roi = image[0:int(height * 0.4), :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # --- 색상 범위 정의 ---
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))

    lower_yellow = np.array([15, 80, 80]) # 노란불도 red로 간주할 것이므로 red_mask에 포함 가능
    upper_yellow = np.array([40, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_green = np.array([40, 100, 100])
    upper_green = np.array([80, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    def has_valid_light(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h if h > 0 else 0
            if area > 100 and 0.5 < aspect_ratio < 1.5: # 조건은 필요시 조정
                return True
        return False

    if has_valid_light(red_mask) or has_valid_light(yellow_mask): # 노란불도 빨간불로 처리
        return 'red'
    
    if has_valid_light(green_mask):
        return 'green'
        
    return "none" # 아무것도 감지되지 않으면 "none" 반환


# =============================================
# 차선을 인식하고 데이터를 추출하는 함수 (수정됨)
# =============================================
def get_line_mask():
    # 이 함수에서 사용할 전역 변수들을 선언합니다.
    global image, last_known_left_lines, last_known_right_lines

    # 1. 차선 마스크 생성 (이전과 동일)
    if image.size == 0:
        return np.zeros((480, 640), dtype=np.uint8), [], [], 320

    # ... (hsv 변환 및 마스크 생성 코드는 이전과 동일) ...
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)
    combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

    height, width = combined_mask.shape
    roi_start_y = height // 2 + 50
    
    roi_mask = combined_mask[roi_start_y:, :]
    
    contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cleaned_roi = np.zeros_like(roi_mask)
    for cnt in contours:
        if cv2.contourArea(cnt) > 50:
            cv2.drawContours(cleaned_roi, [cnt], -1, 255, -1)
            
    final_mask = np.zeros_like(combined_mask)
    final_mask[roi_start_y:, :] = cleaned_roi
    
    # 2. 데이터 추출 (기존과 동일)
    center = width // 2
    row_at_center = cleaned_roi[cleaned_roi.shape[0] // 2]
    contours_vis, _ = cv2.findContours(row_at_center.reshape(1, -1), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx_list = []
    for cnt in contours_vis:
        x, _, w, _ = cv2.boundingRect(cnt)
        cx = x + w // 2
        cx_list.append(cx)

    # 현재 프레임에서 감지된 왼쪽/오른쪽 차선
    left_lines = sorted([cx for cx in cx_list if cx < center])
    right_lines = sorted([cx for cx in cx_list if cx > center])

    # ▼▼▼ 핵심 로직 수정 부분 ▼▼▼
    # 3. 차선 데이터 보정 및 메모리 업데이트
    
    # 만약 현재 프레임에서 차선이 감지되었다면, 'last_known' 변수를 최신 정보로 업데이트
    if left_lines:
        last_known_left_lines = left_lines
    if right_lines:
        last_known_right_lines = right_lines

    # 만약 현재 프레임에서 한쪽 차선이 감지되지 않았다면, 기억해 둔 값으로 보정
    if not left_lines and last_known_left_lines:
        print("[INFO] 왼쪽 차선 없음. 마지막 위치로 복원.")
        left_lines = last_known_left_lines

    if not right_lines and last_known_right_lines:
        print("[INFO] 오른쪽 차선 없음. 마지막 위치로 복원.")
        right_lines = last_known_right_lines
    # ▲▲▲ 핵심 로직 수정 부분 ▲▲▲

    # 4. 시각화 (보정된 데이터를 사용)
    roi_color = cv2.cvtColor(cleaned_roi, cv2.COLOR_GRAY2BGR)
    center_y = roi_color.shape[0] // 2

    # 보정된 차선(실제+기억)을 모두 파란 선으로 표시
    for cx in left_lines + right_lines:
        cv2.line(roi_color, (cx, 0), (cx, roi_color.shape[0]), (255, 0, 0), 2)

    cv2.circle(roi_color, (center, center_y), 5, (0, 255, 0), -1)
    cv2.imshow("roi", roi_color)

    # 5. 최종 데이터 반환
    return final_mask, left_lines, right_lines, center


# =============================================
# 데이터를 받아 차선을 따라 주행하는 함수 (수정됨)
# =============================================
def follow_lane(left_lines, right_lines, center):
    global last_angle # 이전 각도 유지를 위해 global 선언 필요
    global mycar_speed

    # 더 이상 이미지 처리/시각화 코드가 필요 없음

    if left_lines and right_lines:
        # 양쪽 차선이 모두 감지된 경우
        left_closest = max(left_lines)
        right_closest = min(right_lines)
        target_cx = (left_closest + right_closest) // 2

        error = target_cx - center
        speed = mycar_speed
        if abs(error) > 100:
            angle = error * 0.5
            mycar_speed = 13
        elif abs(error) < 10:
            angle = error * 0.25
        else:
            angle = error * 0.3
        #angle = error * 0.3
        print("error: ",error,"angle: ",angle)
        print("dr4")
        drive(angle, mycar_speed)
        last_angle = angle  # 현재 조향각을 다음 루프를 위해 저장

    elif left_lines or right_lines:
        # 한쪽 차선만 감지된 경우 (기존 로직 개선)
        # 이전 조향각을 유지하며 주행
        mycar_speed = 30
        print("dr5")
        drive(last_angle, mycar_speed)
        # print("[INFO] 한쪽 차선만 있음 → angle 유지")

    else:
        # 차선이 전혀 감지되지 않은 경우
        # 이전 조향각을 유지하며 감속
        # print("[WARN] No contours found.")
        print("dr6")
        drive(last_angle, 20)

#=============================================
#FSM(유한 상태 기계)의 판단자 역할.현재 센서 상태, 시간 경과 등을 바탕으로 mode 값을 결정함.
#mode 값은 행동 루틴 (vehicle_avoid, recovery, lane_follow 등)을 선택하는 기준이 됨.
#=============================================
vehicle_last_detected_time = 0     # 차량 마지막 감지 시간 저장용 (쿨다운 적용용)
cooldown_duration = 5.0            # 동일 차량에 대해 재진입 방지를 위한 쿨다운 시간 (초 단위)

cone_path_lost_time = None
CONE_LOST_GRACE_PERIOD = 10.0  # 초 단위

def update_mode():
    global mode, vehicle_last_detected_time, avoid_state
    global cone_path_lost_time, g_cone_path_is_valid

    # g_cone_lost_timestamp, CONE_AVOID_GRACE_PERIOD 는 이번 요청에서는 사용 안 함
    if avoid_state is None:
        avoid_state = 'init'

    # 1. 신호등 감지가 최우선
    current_light_signal = detect_traffic_light()
    if current_light_signal == 'red' or current_light_signal == 'green':
        if mode != 'traffic_light':
            print(f"[FSM] Traffic light '{current_light_signal}' -> mode: traffic_light")
        mode = 'traffic_light'
        return

    # 2. 차량/라바콘 감지   
    # [핵심 변경] 현재 모드가 cone_avoid일 때와 아닐 때를 분리하여 판단
    
    if mode == 'cone_avoid':
        if not g_cone_path_is_valid:
            now = time.time()
            if cone_path_lost_time is None:
                cone_path_lost_time = now
            elif now - cone_path_lost_time > CONE_LOST_GRACE_PERIOD:
                print("[FSM] Cone path lost for too long. Switching to lane_follow.")
                mode = 'lane_follow'
                cone_path_lost_time = None
                return
            else:
                # 아직 유예 기간 내이므로 모드 유지
                return

    # 3. 현재 모드가 cone_avoid가 아닐 때, 새로 cone_avoid로 진입할지 판단
    if avoid_state == 'init': # 차량 회피 FSM이 'init' 상태일 때만 다른 장애물 감지
        obstacle = is_obstacle_close_by_lidar()
        if obstacle:
            obj_detected_by_camera = detect_object()
            if obj_detected_by_camera == 'vehicle':
                 if mode != 'vehicle_avoid':
                    mode = 'vehicle_avoid'
                    avoid_state = 'init'  # 여기서 초기화 반드시 수행
                    vehicle_last_detected_time = time.time()
            elif obj_detected_by_camera == 'cone':
                if mode != 'cone_avoid': 
                    print(f"[FSM] Cones detected -> mode: cone_avoid")
                mode = 'cone_avoid'
                return
    
    # 4. 최종 Fallback
    # 위에서 특정 모드로 return 되지 않았다면, 기본 차선 주행 모드로 설정
    if mode not in ['traffic_light', 'vehicle_avoid',  'cone_avoid']:
        # print(f"[FSM] Defaulting to lane_follow. Current mode was: {mode}")
        mode = 'lane_follow'
    elif mode == 'traffic_light' and current_light_signal == "none": # 신호등 통과 후
        print(f"[FSM] Traffic light sequence ended (signal: 'none'). Switching to lane_follow.")
        mode = 'lane_follow'


# =============================================
# 실질적인 메인 함수
# =============================================
def start():
    global vehicle_avoid_start_time
    global motor, image, ranges, mode, avoid_state, lane_mask, mycar_speed
    avoid_state = 'init'
    mode = 'lane_follow'  # 모드 상태 초기화

    print("Start program --------------")

    # =========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    # =========================================
    rospy.init_node('Track_Driver')
    rospy.Subscriber("/usb_cam/image_raw/", Image, usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)

    # =========================================
    # 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    # =========================================
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("/scan", LaserScan)
    print("Lidar Ready ----------")

    # =========================================
    # 라이다 스캔정보에 대한 시각화 준비를 합니다.
    # =========================================
    plt.ion()
    plt.show()
    print("Lidar Visualizer Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")

    # =========================================
    # 메인 루프
    # =========================================
    while not rospy.is_shutdown():

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("original", image)
        #cv2.imshow("gray", gray)

        if ranges is not None:
            angles = np.linspace(0, 2 * np.pi, len(ranges)) + np.pi / 2
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            lidar_points.set_data(x, y)
            fig.canvas.draw_idle()
            plt.pause(0.01)
        
        lane_mask, left_lines, right_lines, center = get_line_mask()    
        
        update_mode ()

        print(f"[MODE] Current Mode: {mode}")  # 현재 모드 출력
        
        if mode == 'traffic_light':  # 신호등 정지 실행
            handle_traffic_light()
        elif mode == 'vehicle_avoid':  # 차량 회피 실행
            handle_vehicle_avoidance()
            if avoid_state == 'plan' or avoid_state == 'init' or avoid_state == 'maintain_distance':
                if avoid_state == 'maintain_distance':
                    maintain_distance()
                    if abs(last_angle) > 10:
                        mycar_speed = 30
                    if abs(last_angle < 5):
                        time.sleep(0.5)
                follow_lane(left_lines, right_lines, center)
        elif mode == 'cone_avoid':  # 라바콘 회피 실행
            follow_cone_lane()
        elif mode == 'lane_follow':  # 정상 주행 실행
            mycar_speed = 30
            follow_lane(left_lines, right_lines, center)

        time.sleep(0.1)
        cv2.waitKey(1)


# =============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
# =============================================
if __name__ == '__main__':
    start()