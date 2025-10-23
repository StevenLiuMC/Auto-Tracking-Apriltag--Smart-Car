# pc_apriltag_controller_snap.py
import argparse, time, requests, cv2, numpy as np
from pupil_apriltags import Detector

# ---------- 控制逻辑 ----------
def decide_motion(cx, width, deadzone_ratio):
    center = 0.5 * width
    dead = deadzone_ratio * width
    errx = cx - center
    if abs(errx) <= dead:
        return "Forward", errx, dead
    return ("Left", errx, dead) if errx < 0 else ("Right", errx, dead)

def send_cmd(sess, esp_host, motion, speed):
    try:
        sess.post(f"http://{esp_host}/cmd",
                  json={"M": motion if motion != "Stop" else "stop_it",
                        "v": int(speed if motion != "Stop" else 0)},
                  timeout=0.3)
    except requests.RequestException:
        pass

def draw_overlay(vis, motion, fps, dets, chosen, deadzone_ratio):
    h, w = vis.shape[:2]
    dz = int(deadzone_ratio * w)
    xL, xR = w//2 - dz, w//2 + dz
    overlay = vis.copy()
    cv2.rectangle(overlay, (xL, 0), (xR, h), (255, 0, 0), -1)  # 半透明 dead zone
    cv2.addWeighted(overlay, 0.15, vis, 0.85, 0, vis)
    cv2.line(vis, (xL, 0), (xL, h), (255, 0, 0), 1)
    cv2.line(vis, (xR, 0), (xR, h), (255, 0, 0), 1)
    cv2.line(vis, (w//2, 0), (w//2, h), (0, 255, 255), 1)      # 中线

    if chosen is not None:
        ptsi = chosen.corners.astype(int)
        cv2.polylines(vis, [ptsi], True, (0, 255, 0), 2)
        cx, cy = map(int, chosen.center)
        cv2.circle(vis, (cx, cy), 4, (0, 0, 255), -1)

    hud = f"Motion: {motion} | det: {len(dets)} | FPS: {fps:.1f}"
    cv2.putText(vis, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

# ---------- 主程序 ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp", default="192.168.4.1", help="ESP32 host/IP")
    ap.add_argument("--deadzone", type=float, default=0.12, help="dead zone ratio of width")
    ap.add_argument("--speed", type=int, default=90, help="speed value for motion")
    ap.add_argument("--id", type=int, default=None, help="track specific tag id")
    ap.add_argument("--hflip", action="store_true", help="horizontally flip the frame")
    ap.add_argument("--resize_w", type=int, default=0, help="resize width before detection (0=off)")
    ap.add_argument("--show", action="store_true", default=True, help="show visualization window")
    ap.add_argument("--rate", type=float, default=15.0, help="max snapshot rate (Hz)")
    # AprilTag 参数（可按需调）
    ap.add_argument("--families", default="tag36h11")
    ap.add_argument("--quad_decimate", type=float, default=2.0)
    ap.add_argument("--quad_sigma", type=float, default=0.0)  # 近距离更友好
    ap.add_argument("--refine_edges", type=int, default=1)
    ap.add_argument("--threads", type=int, default=2)
    args = ap.parse_args()

    detector = Detector(
        families=args.families,
        nthreads=max(1, args.threads),
        quad_decimate=max(1.0, args.quad_decimate),
        quad_sigma=max(0.0, args.quad_sigma),
        refine_edges=bool(args.refine_edges),
    )

    jpg_url = f"http://{args.esp}/jpg"
    sess = requests.Session()  # 复用连接，降低握手开销

    last_motion, last_presence = None, 0
    min_dt = 1.0 / max(1e-3, args.rate)

    # FPS
    t0, fcnt, fps = time.time(), 0, 0.0

    try:
        while True:
            t_req = time.time()

            # 1) 拉一帧快照（短连接/非阻塞）
            try:
                r = sess.get(jpg_url, timeout=1.0)
                if r.status_code != 200:
                    time.sleep(0.02)
                    continue
            except requests.RequestException:
                time.sleep(0.05)
                continue

            # 2) 解码
            jpg = np.frombuffer(r.content, dtype=np.uint8)
            frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # 3) 预处理：镜像/下采样
            if args.hflip:
                frame = cv2.flip(frame, 1)
            if args.resize_w and frame.shape[1] > args.resize_w:
                scale = args.resize_w / frame.shape[1]
                frame_small = cv2.resize(frame, (args.resize_w, int(frame.shape[0]*scale)),
                                         interpolation=cv2.INTER_AREA)
            else:
                frame_small = frame

            gray = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)

            # 4) 检测
            dets = detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

            h, w = gray.shape[:2]
            chosen, best_area = None, -1.0
            for d in dets:
                if args.id is not None and d.tag_id != args.id:
                    continue
                area = cv2.contourArea(np.array(d.corners, dtype=np.float32))
                if area > best_area:
                    best_area, chosen = area, d

            # 5) 生成动作（未检测到→Stop）
            motion = "Stop"
            if chosen is not None:
                cx, cy = chosen.center
                motion, _, _ = decide_motion(cx, w, args.deadzone)

            # 6) 仅在“状态变化”时发送/打印
            presence = 1 if chosen is not None else 0
            if motion != last_motion:
                send_cmd(sess, args.esp, motion, args.speed)
                print(f"[PC] motion change: {last_motion} -> {motion}")
                last_motion = motion
            if presence != last_presence:
                print("[PC] Apriltag Detected" if presence else "[PC] Apriltag Lost")
                last_presence = presence

            # 7) FPS
            fcnt += 1
            if time.time() - t0 >= 1.0:
                fps = fcnt / (time.time() - t0)
                fcnt, t0 = 0, time.time()

            # 8) 可视化（连续显示=“视频感”）
            if args.show:
                vis = frame.copy()
                draw_overlay(vis, motion, fps, dets, chosen, args.deadzone)
                cv2.imshow("AprilTag Control (SNAP)", vis)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break

            # 9) 控制抓取频率
            dt = time.time() - t_req
            if dt < min_dt:
                time.sleep(min_dt - dt)

    finally:
        if args.show:
            cv2.destroyAllWindows()
        sess.close()

if __name__ == "__main__":
    main()
