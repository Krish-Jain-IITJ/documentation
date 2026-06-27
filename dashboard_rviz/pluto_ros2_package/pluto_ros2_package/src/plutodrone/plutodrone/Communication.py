from plutodrone.Protocol import Protocol
from plutodrone.Common import *
import struct

IDLE = 0
HEADER_START = 1
HEADER_M = 2
HEADER_ARROW = 3
HEADER_SIZE = 4
HEADER_CMD = 5
HEADER_ERR = 6

pro = Protocol()


indx = 0
len = 0
checksum = 0
command = 0
payload_size = 0
input_buffer = []
socketSyckLock = 0
socketOpStarted = 0
checksumIndex = 0
recbuf = []

c_state = IDLE
err_rcvd = False
offset = 0
dataSize = 0
i = 0


#Reading the decoding the MSP packets received            
def readFrame():
    global c_state, recbuf, checksum, offset, dataSize, err_rcvd, cmd
    c = readSock(1)
    c = struct.pack('B', c)
    if c_state == IDLE:
        c_state = HEADER_START if c == b'$' else IDLE
    elif c_state == HEADER_START:
        c_state = HEADER_M if c == b'M' else IDLE
    elif c_state == HEADER_M:
        if c == b'>':
            c_state = HEADER_ARROW
        elif c == b'!':
            c_state = HEADER_ERR
        else:
            c_state = IDLE
    elif c_state == HEADER_ARROW or c_state == HEADER_ERR:
        err_rcvd = (c_state == HEADER_ERR)
        dataSize = (ord(c) & 0xFF)
        offset = 0
        if checksum is None:
            checksum = 0
        checksum = 0
        checksum ^= (ord(c) & 0xFF)
        c_state = HEADER_SIZE
    elif c_state == HEADER_SIZE:
        if checksum is None:
            checksum=0
        cmd = (ord(c) & 0xFF)
        checksum ^= (ord(c) & 0xFF)
        c_state = HEADER_CMD
    elif c_state == HEADER_CMD and offset < dataSize:
        checksum ^= (ord(c) & 0xFF)
        if offset is None:
            offset = 0
        pro.inputBuffer[offset] = (ord(c) & 0xFF)
        offset+=1
    elif c_state == HEADER_CMD and offset >= dataSize:
        if (checksum & 0xFF) == (ord(c) & 0xFF):
            if err_rcvd:
                pass
            else:
                pro.bufferIndex = 0
                pro.evaluateCommand(cmd)
        else:
            pass
        c_state = IDLE
