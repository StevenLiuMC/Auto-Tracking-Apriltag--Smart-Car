import cv2
import numpy as np
import time

# ========== 配置 ==========
CAM_INDEX = 0                    # 摄像头序号
APRILTAG_FAMILY = 'tag36h11'     # 常用家族
LOCK_TAG_ID = None               # 固定跟踪的tag id；None=自动锁定首次检测到的id
DRAW_TRAIL_LEN = 200             # 轨迹点数量上限

SHOW_PNP_POSE = False            # 若有相机内参与Tag边长，设 True
TAG_SIZE_METERS = 0.05           # Tag 实际边长（仅在 SHOW_PNP_POSE=True 生效）

# 可选：相机标定参数（示例：假设1280x720；请替换为你的内参）
CAMERA_MATRIX = np.array([[900,   0, 640],
                          [  0, 900, 360],
                          [  0,   0,   1]], dtype=np.float32)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)  # 如果有畸变，替换这里

# ====== Kalman 调参（重点）======
# 观测噪声（像素）：AprilTag中心通常很干净，1~4像素均可尝试
R_MEAS_PX = 1.5

# jerk 白噪声强度（像素/秒^3）：越大→响应更快/更不拖尾；越小→更平滑但更滞后
SIGMA_JERK = 120.0

# 初始协方差（越大=对该维的不确定性越大，便于尽快被数据“校准”）
P_INIT_POS = 4.0
P_INIT_VEL = 400.0
P_INIT_ACC = 1000.0

# 仅用于可视化的“超前预测”时间（用于补偿检测/显示延迟），可设 0.06~0.15s
LEAD_SECONDS = 0.10

# 检测质量门限（过低 margin 易产生离群测量，可丢弃）
MIN_DECISION_MARGIN = 10.0

# ========== pupil-apriltags 检测器 ==========
try:
    from pupil_apriltags import Detector
except ImportError as e:
    raise SystemExit("未安装 pupil-apriltags。请先运行：pip install pupil-apriltags opencv-python numpy") from e

detector = Detector(
    families=APRILTAG_FAMILY,
    nthreads=2,           # 视CPU而定，适当调大
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=True,
    decode_sharpening=0.25,
    debug=False
)

# ========== Kalman（CA：x,y,vx,vy,ax,ay）==========
# 状态维=6，观测维=2（只量到x,y）
kf = cv2.KalmanFilter(6, 2, 0)

# H（观测矩阵）：只观测位置
kf.measurementMatrix = np.array([
    [1,0,0,0,0,0],
    [0,1,0,0,0,0]
], dtype=np.float32)

# R（测量噪声）
kf.measurementNoiseCov = np.diag([R_MEAS_PX**2, R_MEAS_PX**2]).astype(np.float32)

# P（初值协方差）
kf.errorCovPost = np.diag([
    P_INIT_POS, P_INIT_POS,
    P_INIT_VEL, P_INIT_VEL,
    P_INIT_ACC, P_INIT_ACC
]).astype(np.float32)

# 初始状态
kf.statePost = np.zeros((6,1), dtype=np.float32)

def update_F_Q_CA(kf, dt, sigma_jerk):
    """
    常加速度模型（CA）：
    单轴状态 [x, v, a]^T, jerk 为白噪声，离散 Q:
      q = sigma_jerk^2
      Q1 = [[dt^5/20, dt^4/8, dt^3/6],
            [dt^4/8 , dt^3/3, dt^2/2],
            [dt^3/6 , dt^2/2, dt     ]] * q
    二维作 block-diag；F 同理。
    """
    dt2 = dt*dt
    dt3 = dt2*dt
    dt4 = dt2*dt2
    dt5 = dt2*dt3
    q = sigma_jerk * sigma_jerk

    # F（6x6）：二维 block of [[1, dt, 0.5dt^2],[0,1,dt],[0,0,1]]
    F = np.array([
        [1,0, dt,0, 0.5*dt2, 0],
        [0,1, 0, dt, 0, 0.5*dt2],
        [0,0, 1, 0, dt, 0],
        [0,0, 0, 1, 0, dt],
        [0,0, 0, 0, 1, 0],
        [0,0, 0, 0, 0, 1],
    ], dtype=np.float32)
    kf.transitionMatrix = F

    # Q（6x6）：二维 block-diag of Q1
    Q1 = np.array([
        [dt5/20.0, dt4/8.0, dt3/6.0],
        [dt4/8.0 , dt3/3.0, dt2/2.0],
        [dt3/6.0 , dt2/2.0, dt      ]
    ], dtype=np.float32) * q

    Q = np.zeros((6,6), dtype=np.float32)
    # x block
    Q[0:3,0:3] = Q1
    # y block
    Q[3:6,3:6] = Q1
    kf.processNoiseCov[:] = Q

# ========== 轨迹点缓存 ==========
meas_trail, filt_trail, pred_trail, lead_trail = [], [], [], []

def estimate_pose_solvepnp(corners, camera_matrix, dist_coeffs, tag_size):
    s = tag_size / 2.0
    objp = np.array([[-s, -s, 0],
                     [ s, -s, 0],
                     [ s,  s, 0],
                     [-s,  s, 0]], dtype=np.float32)  # A,B,C,D
    imgp = corners.astype(np.float32)
    success, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    return success, rvec, tvec

def predict_ahead_from_state(x_post, lead_dt):
    """基于当前状态做一次“超前”前推（不改动KF内部状态）"""
    # 手工按 CA 模型前推（更直观），也可以复用 F(lead_dt)
    x, y, vx, vy, ax, ay = [float(v) for v in x_post.ravel()]
    x_lead = x + vx*lead_dt + 0.5*ax*(lead_dt**2)
    y_lead = y + vy*lead_dt + 0.5*ay*(lead_dt**2)
    return x_lead, y_lead

# ========== 主循环 ==========
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise SystemExit(f"无法打开摄像头 {CAM_INDEX}")

ret, frame = cap.read()
if not ret:
    raise SystemExit("无法从摄像头读取第一帧。")

h0, w0 = frame.shape[:2]
print(f"Camera opened: {w0}x{h0}")

prev_time = time.time()
locked_id = LOCK_TAG_ID

while True:
    ret, frame = cap.read()
    if not ret:
        break

    now = time.time()
    dt = max(1e-3, now - prev_time)  # 避免0
    prev_time = now

    # 更新 F、Q（随 dt）
    update_F_Q_CA(kf, dt, SIGMA_JERK)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    # 选择跟踪的Tag
    chosen_det = None
    if detections:
        if locked_id is None:
            # 首次锁定：选 decision_margin 最大者
            candidate = max(detections, key=lambda d: d.decision_margin)
            if candidate.decision_margin >= MIN_DECISION_MARGIN:
                chosen_det = candidate
                locked_id = int(candidate.tag_id)
        else:
            for d in detections:
                if int(d.tag_id) == int(locked_id) and d.decision_margin >= MIN_DECISION_MARGIN:
                    chosen_det = d
                    break

    # 先验预测
    pred = kf.predict()
    px, py = float(pred[0]), float(pred[1])

    had_measurement = False
    pose_text = ""

    if chosen_det is not None:
        corners = np.array(chosen_det.corners, dtype=np.float32)
        cx, cy = float(chosen_det.center[0]), float(chosen_det.center[1])

        # 后验校正
        measurement = np.array([[cx], [cy]], dtype=np.float32)
        est = kf.correct(measurement)
        ex, ey = float(est[0]), float(est[1])
        had_measurement = True

        # 画tag框与中心
        for i in range(4):
            pt1 = tuple(corners[i].astype(int))
            pt2 = tuple(corners[(i+1) % 4].astype(int))
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 4, (0, 255, 0), -1)
        cv2.putText(frame, f"id:{int(chosen_det.tag_id)}  m:{chosen_det.decision_margin:.1f}",
                    (int(cx)+8, int(cy)-8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2, cv2.LINE_AA)

        # 轨迹记录
        meas_trail.append((int(cx), int(cy)))
        filt_trail.append((int(ex), int(ey)))
        if len(meas_trail) > DRAW_TRAIL_LEN: meas_trail.pop(0)
        if len(filt_trail) > DRAW_TRAIL_LEN: filt_trail.pop(0)

        # 可选：位姿估计
        if SHOW_PNP_POSE:
            ok, rvec, tvec = estimate_pose_solvepnp(corners, CAMERA_MATRIX, DIST_COEFFS, TAG_SIZE_METERS)
            if ok:
                distance = np.linalg.norm(tvec)
                pose_text = f"Z~{tvec[2][0]:.3f} m, |t|={distance:.3f} m"
                axis = np.float32([[0.02,0,0],[0,0.02,0],[0,0,0.02]]).reshape(-1,3)
                imgpts, _ = cv2.projectPoints(axis, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS)
                c = (int(cx), int(cy))
                cv2.line(frame, c, tuple(imgpts[0].ravel().astype(int)), (255,0,0), 2)
                cv2.line(frame, c, tuple(imgpts[1].ravel().astype(int)), (0,255,0), 2)
                cv2.line(frame, c, tuple(imgpts[2].ravel().astype(int)), (0,0,255), 2)

    # 若本帧没有测量，记录先验预测（红线）
    if not had_measurement:
        pred_trail.append((int(px), int(py)))
        if len(pred_trail) > DRAW_TRAIL_LEN: pred_trail.pop(0)

    # —— 超前预测（橙色）——
    if LEAD_SECONDS > 0.0:
        lead_x, lead_y = predict_ahead_from_state(kf.statePost, LEAD_SECONDS)
        lead_trail.append((int(lead_x), int(lead_y)))
        if len(lead_trail) > DRAW_TRAIL_LEN: lead_trail.pop(0)
        cv2.circle(frame, (int(lead_x), int(lead_y)), 5, (0,165,255), -1)

    # 画轨迹
    for i in range(1, len(meas_trail)):
        cv2.line(frame, meas_trail[i-1], meas_trail[i], (0, 200, 0), 2)       # 绿色：测量
    for i in range(1, len(filt_trail)):
        cv2.line(frame, filt_trail[i-1], filt_trail[i], (200, 200, 255), 2)   # 浅蓝：滤波
    for i in range(1, len(pred_trail)):
        cv2.line(frame, pred_trail[i-1], pred_trail[i], (0, 0, 255), 2)       # 红色：纯预测（无测量帧）
    for i in range(1, len(lead_trail)):
        cv2.line(frame, lead_trail[i-1], lead_trail[i], (0,165,255), 2)       # 橙色：超前预测

    # 显示信息
    cv2.putText(frame, f"Locked ID: {locked_id if locked_id is not None else 'auto'}",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"dt: {dt*1000:.1f} ms", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
    if SHOW_PNP_POSE and pose_text:
        cv2.putText(frame, pose_text, (10, 76),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

    cv2.imshow("AprilTag + Kalman (CA + Q(dt) + Lead)", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        break
    elif key == ord('r'):
        # 重置
        locked_id = None
        meas_trail.clear(); filt_trail.clear(); pred_trail.clear(); lead_trail.clear()
        kf.statePost[:] = 0
        kf.errorCovPost[:] = np.diag([
            P_INIT_POS, P_INIT_POS,
            P_INIT_VEL, P_INIT_VEL,
            P_INIT_ACC, P_INIT_ACC
        ]).astype(np.float32)

cap.release()
cv2.destroyAllWindows()
