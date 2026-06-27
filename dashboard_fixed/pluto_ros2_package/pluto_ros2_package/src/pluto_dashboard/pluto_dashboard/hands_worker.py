#!/usr/bin/env python3
"""
MediaPipe hand-detection worker.
Runs inside ~/mediapipe_env (NumPy 1.x) so it never conflicts with
the ROS dashboard process (NumPy 2.x / opencv 4.13).

Protocol (over stdin/stdout, all binary):
  PARENT → WORKER :  4-byte big-endian uint32 (byte length N)  +  N bytes JPEG
  WORKER → PARENT :  4-byte big-endian uint32 (byte length M)  +  M bytes JPEG (annotated)

If detection fails for a frame the original JPEG is returned unchanged.
"""
import sys
import struct

import cv2
import numpy as np
import mediapipe as mp

mp_hands   = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

stdin  = sys.stdin.buffer
stdout = sys.stdout.buffer

def read_frame():
    hdr = stdin.read(4)
    if len(hdr) < 4:
        return None
    n = struct.unpack('>I', hdr)[0]
    data = b''
    while len(data) < n:
        chunk = stdin.read(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def write_frame(data: bytes):
    stdout.write(struct.pack('>I', len(data)))
    stdout.write(data)
    stdout.flush()

while True:
    jpg_bytes = read_frame()
    if jpg_bytes is None:
        break

    try:
        arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            write_frame(jpg_bytes)
            continue

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        if results.multi_hand_landmarks:
            h, w = bgr.shape[:2]
            for hand_lm in results.multi_hand_landmarks:
                pts = []
                for lm in hand_lm.landmark:
                    pts.append([int(lm.x * w), int(lm.y * h)])
                hull = cv2.convexHull(
                    np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
                )
                overlay = bgr.copy()
                cv2.fillPoly(overlay, [hull], (0, 200, 80))
                cv2.addWeighted(overlay, 0.30, bgr, 0.70, 0, bgr)
                mp_drawing.draw_landmarks(
                    bgr, hand_lm, mp_hands.HAND_CONNECTIONS,
                    mp_drawing.DrawingSpec(color=(0, 255, 80), thickness=2, circle_radius=3),
                    mp_drawing.DrawingSpec(color=(0, 200, 60), thickness=2),
                )

        ok, out_jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 60])
        write_frame(out_jpg.tobytes() if ok else jpg_bytes)

    except Exception as e:
        sys.stderr.write(f'worker error: {e}\n')
        sys.stderr.flush()
        write_frame(jpg_bytes)
