import sys
sys.path.insert(0, 'install/pluto_camera_sense/lib/python3.12/site-packages')

import av
print(f"PyAV version: {av.__version__}")

from pluto_camera_sense.lwdrone import LWDrone
from pluto_camera_sense.defaults import CAM_IP

print(f"Connecting to {CAM_IP}...")
drone = LWDrone(ip=CAM_IP)
print("Connected!")

codec = av.CodecContext.create('h264', 'r')
fed = 0
decoded = 0

print("Starting stream — feeding 60 frames...")
for vframe in drone.start_video_stream(False):
    raw = bytes(vframe.frame_bytes)
    pkt = av.Packet(raw)
    fed += 1
    try:
        frames = codec.decode(pkt)
        for f in frames:
            decoded += 1
            print(f"  DECODED frame {decoded}: {f.width}x{f.height}")
    except Exception as e:
        print(f"  decode error frame {fed}: {e}")
    print(f"  Fed {fed} NAL chunks, decoded {decoded} so far...")
    if fed >= 60:
        break

drone.stop_video_stream()
print(f"\n=== RESULT: Fed {fed}, Decoded {decoded} ===")
