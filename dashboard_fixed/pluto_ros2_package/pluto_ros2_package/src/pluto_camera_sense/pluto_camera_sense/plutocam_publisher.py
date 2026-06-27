#!/usr/bin/env python3
"""
plutocam_publisher.py — Fixed version.

Key fixes vs original:
1. cv_bridge REMOVED — broken by NumPy 2.x. Image message built manually.
2. FFmpeg stdout pipe enlarged to 4 MB via F_SETPIPE_SZ so FFmpeg never
   stalls writing decoded frames while Python is busy publishing to ROS.
3. Frame read loop uses readinto() + memoryview — zero-copy, avoids the
   byte-concatenation overhead that caused backpressure.
4. FFmpeg flags cleaned up: fflags nobuffer + low_delay for minimal latency.
"""

import sys
import os
import fcntl
import signal
import argparse
import subprocess
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .lwdrone import LWDrone
from .defaults import CAM_IP


class PlutoCameraNode(Node):
    def __init__(self, ip, low_def=False, display=False, out_file='-'):
        super().__init__('pluto_camera_publisher')
        self.ip = ip
        self.low_def = low_def
        self.display = display
        self.out_file = out_file
        self.width  = 1280 if low_def else 1920
        self.height = 720  if low_def else 1080
        self.frame_size = self.width * self.height * 3  # BGR24

        # QoS depth=1: always publish the latest frame, never queue stale ones
        self.image_pub = self.create_publisher(Image, 'plutocamera/image_raw', 1)

        self.ffmpeg_decoder = None
        self.ffplay_process  = None
        self.drone   = None
        self.running = False

        signal.signal(signal.SIGINT, self.signal_handler)

    # ------------------------------------------------------------------ #
    def setup_ffmpeg_decoder(self):
        cmd = [
            'ffmpeg',
            '-fflags', 'nobuffer',   # no input buffering → minimal latency
            '-flags',  'low_delay',  # low-delay decode mode
            '-i', 'pipe:0',
            '-vf', 'vflip,hflip',    # 180° flip
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-vcodec', 'rawvideo',
            '-',
        ]
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,               # unbuffered I/O on the Python side
        )
        # Enlarge the kernel pipe buffer for stdout so FFmpeg can keep
        # writing decoded frames while Python is busy publishing to ROS.
        # Without this, FFmpeg stalls after ~1 frame (64 KB default pipe).
        try:
            F_SETPIPE_SZ = 1031      # Linux-specific fcntl constant
            fcntl.fcntl(proc.stdout.fileno(), F_SETPIPE_SZ, 4 * 1024 * 1024)
        except Exception:
            pass                     # Non-Linux — silently ignore
        return proc

    # ------------------------------------------------------------------ #
    def setup_ffplay_display(self):
        cmd = ['ffplay', '-sync', 'ext', '-framedrop', '-f', 'h264', '-i', 'pipe:0']
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)

    # ------------------------------------------------------------------ #
    def signal_handler(self, sig, frame):
        print('\nShutting down...', file=sys.stderr)
        self.running = False

    # ------------------------------------------------------------------ #
    def process_frames(self):
        """Read BGR24 frames from FFmpeg stdout and publish as ROS Image."""
        buf  = bytearray(self.frame_size)
        view = memoryview(buf)

        while self.running and self.ffmpeg_decoder.poll() is None:
            try:
                # Accumulate exactly frame_size bytes using readinto (zero-copy)
                pos = 0
                while pos < self.frame_size and self.running:
                    n = self.ffmpeg_decoder.stdout.readinto(view[pos:])
                    if not n:
                        self.get_logger().warn('FFmpeg stdout closed')
                        return
                    pos += n

                if pos != self.frame_size:
                    continue

                # Build ROS Image message without cv_bridge
                ros_image = Image()
                ros_image.header.stamp    = self.get_clock().now().to_msg()
                ros_image.header.frame_id = 'camera'
                ros_image.height     = self.height
                ros_image.width      = self.width
                ros_image.encoding   = 'bgr8'
                ros_image.is_bigendian = 0
                ros_image.step       = self.width * 3
                ros_image.data       = bytes(buf)
                self.image_pub.publish(ros_image)

            except Exception as e:
                self.get_logger().warn(f'Frame processing error: {e}')
                break

    # ------------------------------------------------------------------ #
    def run(self):
        try:
            self.ffmpeg_decoder = self.setup_ffmpeg_decoder()

            if self.display:
                self.ffplay_process = self.setup_ffplay_display()

            print(f'Connecting to PlutoCamera at {self.ip}...', file=sys.stderr)
            self.drone = LWDrone(ip=self.ip)
            print(f'Connected to PlutoCamera at {self.ip}', file=sys.stderr)

            resolution = '720p' if self.low_def else '1080p'
            print(f'Starting {resolution} video stream... (Press Ctrl+C to stop)', file=sys.stderr)

            self.running = True

            process_thread = threading.Thread(target=self.process_frames, daemon=True)
            process_thread.start()

            out_file_handle = None
            if self.out_file != '-':
                out_file_handle = open(self.out_file, 'wb')

            try:
                for frame in self.drone.start_video_stream(not self.low_def):
                    if not self.running:
                        break
                    frame_data = frame.frame_bytes

                    try:
                        self.ffmpeg_decoder.stdin.write(frame_data)
                        self.ffmpeg_decoder.stdin.flush()
                    except (BrokenPipeError, OSError) as e:
                        print(f'Error writing to FFmpeg: {e}', file=sys.stderr)
                        break

                    if out_file_handle is not None:
                        out_file_handle.write(frame_data)
                        out_file_handle.flush()

                    if self.display and self.ffplay_process and self.ffplay_process.poll() is None:
                        try:
                            self.ffplay_process.stdin.write(frame_data)
                            self.ffplay_process.stdin.flush()
                        except (BrokenPipeError, OSError):
                            print('\nDisplay closed', file=sys.stderr)
                            self.display = False

            finally:
                if out_file_handle:
                    out_file_handle.close()

            process_thread.join(timeout=2.0)
            if process_thread.is_alive():
                print('Warning: Frame processing thread did not exit cleanly', file=sys.stderr)

        except Exception as e:
            print(f'Error: {e}', file=sys.stderr)
            return 1
        finally:
            self.cleanup()

    # ------------------------------------------------------------------ #
    def cleanup(self):
        try:
            if self.drone:
                self.drone.stop_video_stream()
        except Exception:
            pass
        try:
            if self.ffplay_process and self.ffplay_process.poll() is None:
                self.ffplay_process.terminate()
                self.ffplay_process.wait()
        except Exception:
            pass


# ------------------------------------------------------------------ #
def main():
    parser = argparse.ArgumentParser(description='Stream PlutoCamera video and publish as ROS topic')
    parser.add_argument('--ip',      default=CAM_IP)
    parser.add_argument('--low-def', action='store_true', help='720p instead of 1080p')
    parser.add_argument('--display', '-d', action='store_true')
    parser.add_argument('--out-file', '-o', default='-')
    args = parser.parse_args()

    rclpy.init(args=None)
    node = PlutoCameraNode(
        ip=args.ip,
        low_def=args.low_def,
        display=args.display,
        out_file=args.out_file,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    print('Stream stopped', file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())
