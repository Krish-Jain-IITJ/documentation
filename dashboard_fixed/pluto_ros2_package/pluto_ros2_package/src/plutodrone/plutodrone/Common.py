import socket
import errno
import select

PORT = 23
IP_ADDRESS = "192.168.4.1"
CAMERA_PORT = 9060
CAMERA_IP_ADDRESS = "192.168.0.1"

sock = None

def _try_connect(host, port, timeout=2.0):
    """Open a TCP socket to (host, port) with a bounded timeout.

    Returns the connected socket on success, else None. Used so connectSock
    can probe both the drone-direct AP and the camera-bridged AP without
    hanging for 7 seconds on the first miss.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        s.settimeout(None)
        return s
    except Exception as e:
        try: s.close()
        except Exception: pass
        return None


def connectSock():
    """Connect to the flight controller, camera-aware.

    Two configurations:
      - no camera module: FC is directly on 192.168.4.1:23
      - camera module attached: FC is reachable at 192.168.0.1:9060 (camera
        module acts as an MSP relay). Drone AP is off in this case, so the
        client must already be on the camera's WIFI-1080p-... SSID.

    We try camera-mode first (since the camera AP is the common "all-in-one"
    setup), then fall back to direct.
    """
    global sock
    import os
    override = os.environ.get('PLUTO_HOST')  # e.g. "192.168.0.1:9060"
    candidates = []
    if override:
        try:
            h, p = override.split(':')
            candidates.append((h.strip(), int(p)))
        except Exception:
            print(f'PLUTO_HOST malformed: {override!r}')
    candidates.extend([
        (CAMERA_IP_ADDRESS, CAMERA_PORT),  # camera-bridged
        (IP_ADDRESS,        PORT),         # drone direct
    ])
    for host, port in candidates:
        print(f'[plutonode] trying {host}:{port} …')
        s = _try_connect(host, port, timeout=2.0)
        if s is not None:
            sock = s
            print(f'[plutonode] connected to {host}:{port}')
            return True
    print('Cannot connect to Pluto — tried camera ({}:{}) and direct ({}:{}).'
          .format(CAMERA_IP_ADDRESS, CAMERA_PORT, IP_ADDRESS, PORT))
    print('Check Wi-Fi: join WIFI-1080p-… (camera) or the drone direct AP.')
    exit(0)

def writeSock(data):
    global sock
    try:
        data = bytes(data)
        # print(data)
        bytes_sent = sock.send(data)
        socketSyncLock = 1
        return bytes_sent
    except socket.error as e:
        print("Error while writing to socket:", e)
        return -1

def readSock(count):
    try:
        data = sock.recv(count)
        if data:
            val = data[0]
            return val
        else:
            return 0  # Connection closed
    except socket.error as e:
        print("Error while reading from socket:", e)
        return -1
    