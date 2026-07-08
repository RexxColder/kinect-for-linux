#!/usr/bin/env python3
"""Kinect Skeleton Tracker - MediaPipe Pose 33 landmarks, clamped to image."""
import mmap, struct, os, time
import numpy as np
import mediapipe as mp
import cv2

SHM_MAGIC = 0x4B345750
SHM_RING_SIZE = 3
MAX_SK = 10
NKP = 33

# High-confidence keypoint indices for upper body (seated-friendly)
# 0=nose, 11=left_shoulder, 12=right_shoulder, 13=left_elbow, 14=right_elbow,
# 15=left_wrist, 16=right_wrist, 23=left_hip, 24=right_hip
UPPER_BODY = {0, 11, 12, 13, 14, 15, 16, 23, 24}
# Lower body keypoints that are unreliable when seated
LOWER_BODY = {25, 26, 27, 28, 29, 30, 31, 32}

def open_shm(name):
    path = f"/dev/shm/{name}"
    if not os.path.exists(path): return None, 0
    fd = os.open(path, os.O_RDONLY)
    sz = os.fstat(fd).st_size
    mm = mmap.mmap(fd, sz, mmap.MAP_SHARED, mmap.PROT_READ)
    os.close(fd)
    return mm, sz

def read_frame(mm, size):
    d = mm[:32]
    if struct.unpack_from('<I', d, 0)[0] != SHM_MAGIC: return None, 0, 0, 0
    fs = struct.unpack_from('<I', d, 4)[0]
    w, h = struct.unpack_from('<II', d, 8)
    bpp = struct.unpack_from('<I', d, 16)[0]
    widx = struct.unpack_from('<I', d, 20)[0]
    seq = struct.unpack_from('<I', d, 28)[0]
    off = 32 + ((widx + SHM_RING_SIZE - 1) % SHM_RING_SIZE) * fs
    return np.frombuffer(mm[off:off+fs], dtype=np.uint8).reshape((h, w, bpp)), w, h, seq

def write_shm(skeletons, w, h):
    path = "/dev/shm/k4w_skeleton"
    sk_sz = 16 + 4 + NKP * 12
    total = 32 + MAX_SK * sk_sz
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o666)
    os.ftruncate(fd, total)
    mm = mmap.mmap(fd, total, mmap.MAP_SHARED, mmap.PROT_WRITE)
    os.close(fd)
    struct.pack_into('<IIIIq', mm, 0, SHM_MAGIC, w, h, min(len(skeletons), MAX_SK), int(time.time()*1000))
    off = 32
    for sk in skeletons[:MAX_SK]:
        struct.pack_into('<ffff', mm, off, *sk['bbox']); off += 16
        struct.pack_into('<f', mm, off, sk['conf']); off += 4
        for kp in sk['kps'][:NKP]:
            struct.pack_into('<iif', mm, off, kp[0], kp[1], kp[2]); off += 12
        for _ in range(NKP - len(sk['kps'])):
            struct.pack_into('<iif', mm, off, 0, 0, 0.0); off += 12
    mm.flush(); mm.close()

def main():
    print("Starting Kinect Skeleton Tracker (MediaPipe Pose Lite)...", flush=True)
    model_path = "/home/rexx/Proyectos/k4w-suite/models/pose_landmarker_lite.task"
    if not os.path.exists(model_path):
        model_path = "/home/rexx/Proyectos/k4w-suite/models/pose_landmarker_heavy.task"
        print(f"Lite not found, using heavy: {model_path}", flush=True)
    else:
        print(f"Using lite model: {model_path}", flush=True)
    model = mp.tasks.vision.PoseLandmarker.create_from_options(
        mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model_path),
            running_mode=mp.tasks.vision.RunningMode.VIDEO, num_poses=1,
            min_pose_detection_confidence=0.6, min_pose_presence_confidence=0.6,
            min_tracking_confidence=0.6))
    print("Ready", flush=True)
    last_seq, fps_c, fps_t = 0, 0, time.time()
    frame_interval = 1.0 / 10.0  # 10fps inference
    while True:
        t_start = time.time()
        mm, sz = open_shm("k4w_video")
        if mm is None: time.sleep(0.5); continue
        frame, w, h, seq = read_frame(mm, sz); mm.close()
        if frame is None or seq == last_seq: time.sleep(0.005); continue
        last_seq = seq
        small = cv2.resize(frame, (320, 240))
        bgr = cv2.cvtColor(small, cv2.COLOR_RGB2BGR)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=bgr)
        result = model.detect_for_video(mp_img, int(time.time()*1000))
        skeletons = []
        if result.pose_landmarks:
            for plm in result.pose_landmarks:
                # Count high-confidence upper body keypoints
                upper_vis = sum(1 for i in UPPER_BODY if plm[i].visibility > 0.5)
                # Need at least 5 upper body points visible to consider it a valid detection
                if upper_vis < 5:
                    continue
                # Overall confidence = mean visibility of upper body
                upper_confs = [plm[i].visibility for i in UPPER_BODY if plm[i].visibility > 0.5]
                mean_conf = sum(upper_confs) / len(upper_confs) if upper_confs else 0.0
                if mean_conf < 0.4:
                    continue
                sk = {'kps': [], 'bbox': [0, 0, w, h], 'conf': mean_conf}
                for i in range(NKP):
                    lm = plm[i]
                    vis = lm.visibility
                    # Lower body keypoints get confidence zeroed if upper body is low
                    # (person is likely seated or out of frame below)
                    if i in LOWER_BODY and upper_vis < 7:
                        vis = 0.0
                    # Zero out keypoints with very low visibility
                    if vis < 0.3:
                        vis = 0.0
                    x = max(0, min(int(lm.x * w), w - 1)) if vis > 0.0 else 0
                    y = max(0, min(int(lm.y * h), h - 1)) if vis > 0.0 else 0
                    sk['kps'].append([x, y, vis])
                xs = [k[0] for k in sk['kps'] if k[2] > 0.4]
                ys = [k[1] for k in sk['kps'] if k[2] > 0.4]
                if xs and ys:
                    sk['bbox'] = [min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)]
                else:
                    continue
                skeletons.append(sk)
        write_shm(skeletons, w, h)
        fps_c += 1
        if time.time() - fps_t >= 1.0:
            print(f"FPS:{fps_c} Sk:{len(skeletons)}", flush=True)
            fps_c = 0
            fps_t = time.time()
        elapsed = time.time() - t_start
        sleep_time = frame_interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

if __name__ == "__main__":
    main()
