from plutodrone.Common import * 

MSP_FC_VERSION=3
MSP_RAW_IMU=102
MSP_RC = 105
MSP_ATTITUDE=108
MSP_ALTITUDE=109
MSP_ANALOG=110
MSP_SET_RAW_RC=200
MSP_PID=112
MSP_SET_PID=202
MSP_ACC_CALIBRATION=205
MSP_MAG_CALIBRATION=206
MSP_SET_MOTOR=214
MSP_SET_ACC_TRIM=239
MSP_ACC_TRIM=240
MSP_EEPROM_WRITE = 250
MSP_SET_POS= 216
MSP_SET_COMMAND = 217

IDLE = 0
HEADER_START = 1
HEADER_M = 2
HEADER_ARROW = 3
HEADER_SIZE = 4
HEADER_CMD = 5
HEADER_ERR = 6

MSP_HEADER="$M<"

alt = 0.0
roll = 0
pitch = 0
yaw = 0
battery = 0.0     # volts (V1: vBatComp/1000, legacy: vbat/10)
rssi = 0          # legacy only; V1 doesn't send it
# ── Magis v2 INA219 BMS (MSP Protocol V1 enhanced) ──
mamp_raw = 0      # instantaneous current draw (mA)
mah_drawn = 0     # charge drawn since boot
mah_remain = 0    # estimated charge remaining
soc = 0           # state-of-charge percentage (0..100)
auto_land_mode = 0  # 1 = firmware decided to auto-land
acc_x = 0.0
acc_y = 0.0
acc_z = 0.0
gyro_x = 0.0
gyro_y = 0.0
gyro_z = 0.0
mag_x = 0.0
mag_y = 0.0
mag_z = 0.0

FC_versionMajor = 0
FC_versionMinor = 0
FC_versionPatchLevel = 0

trim_roll = 0
trim_pitch = 0

rc_throttle = 1500
rc_roll = 1500
rc_pitch = 1500
rc_yaw = 1500
rc_aux1 = 1500
rc_aux2 = 1500
rc_aux3 = 1500
rc_aux4 = 1500

class Protocol():
    inputBuffer = bytearray(1024) # used to store data from packets
    bufferIndex = 0

    def read8(self):
        val = self.inputBuffer[self.bufferIndex] & 0xFF
        self.bufferIndex += 1
        return val

    def read16u(self):
        lo = self.inputBuffer[self.bufferIndex] & 0xFF
        hi = self.inputBuffer[self.bufferIndex + 1] & 0xFF
        self.bufferIndex += 2
        return lo | (hi << 8)

    def read16(self):
        v = self.read16u()
        return v - 0x10000 if v >= 0x8000 else v

    def read32u(self):
        b0 = self.inputBuffer[self.bufferIndex] & 0xFF
        b1 = self.inputBuffer[self.bufferIndex + 1] & 0xFF
        b2 = self.inputBuffer[self.bufferIndex + 2] & 0xFF
        b3 = self.inputBuffer[self.bufferIndex + 3] & 0xFF
        self.bufferIndex += 4
        return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)

    def read32(self):
        v = self.read32u()
        return v - 0x100000000 if v >= 0x80000000 else v

    def evaluateCommand(self, command):
        global roll, pitch, yaw, battery, rssi, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, alt, rc_aux1, rc_aux2, rc_aux3, rc_aux4, rc_throttle, rc_pitch, rc_yaw, rc_roll
        global FC_versionMajor, FC_versionMinor, FC_versionPatchLevel, trim_roll, trim_pitch
        global mamp_raw, mah_drawn, mah_remain, soc, auto_land_mode
    
        if command == MSP_FC_VERSION:
            FC_versionMajor = self.read8()
            FC_versionMinor = self.read8()
            FC_versionPatchLevel = self.read8()

        elif command == MSP_RAW_IMU:
            # MSP RAW_IMU returns 9× signed int16: accel, gyro, mag.
            # Accel is raw (≈512 per g on Pluto); scaling to g done downstream.
            # Gyro / 8 gives deg/s (MultiWii convention).
            acc_x = self.read16()
            acc_y = self.read16()
            acc_z = self.read16()
            gyro_x = self.read16() / 8.0
            gyro_y = self.read16() / 8.0
            gyro_z = self.read16() / 8.0
            mag_x = self.read16()
            mag_y = self.read16()
            mag_z = self.read16()

        elif command == MSP_ATTITUDE:
            # roll/pitch: signed int16 decidegrees (−1800..1800). yaw: signed int16 degrees.
            # Values stay raw here; dashboard divides roll/pitch by 10.
            roll = self.read16()
            pitch = self.read16()
            yaw = self.read16()

        elif command == MSP_ALTITUDE:
            # Signed int32 in cm. Dashboard divides by 100 for m.
            alt = self.read32()
            # MSP spec also has int16 vario (cm/s) after alt; ignore for now.

        elif command == MSP_ANALOG:
            # Magis v2 uses MSP Protocol Version 1 (enhanced, INA219 BMS):
            #   uint16 vBatComp (mV)  · uint16 mAmpRaw (mA)
            #   uint16 mAhDrawn       · uint16 mAhRemain
            #   uint8  soc (%)        · uint8  autoLandMode
            battery        = self.read16u() / 1000.0
            mamp_raw       = self.read16u()
            mah_drawn      = self.read16u()
            mah_remain     = self.read16u()
            soc            = self.read8()
            auto_land_mode = self.read8()

        elif command == MSP_ACC_TRIM:
            trim_pitch = self.read16()
            trim_roll = self.read16()

        elif command == MSP_RC:
            rc_roll     = self.read16u()
            rc_pitch    = self.read16u()
            rc_yaw      = self.read16u()
            rc_throttle = self.read16u()
            rc_aux1     = self.read16u()
            rc_aux2     = self.read16u()
            rc_aux3     = self.read16u()
            rc_aux4     = self.read16u()

        else:
            # Handle other cases here if needed
            pass

        
    def returnData(self):
        global roll, pitch, yaw, battery, rssi, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, alt, rc_aux1, rc_aux2, rc_aux3, rc_aux4, rc_throttle, rc_pitch, rc_yaw, rc_roll
        global FC_versionMajor, FC_versionMinor, FC_versionPatchLevel, trim_roll, trim_pitch
        return roll, pitch, yaw, battery, rssi, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, alt, rc_aux1, rc_aux2, rc_aux3, rc_aux4, rc_throttle, rc_pitch, rc_yaw, rc_roll

    def returnBms(self):
        """Magis v2 INA219 BMS: (current_ma, mah_drawn, mah_remain, soc, auto_land)."""
        global mamp_raw, mah_drawn, mah_remain, soc, auto_land_mode
        return mamp_raw, mah_drawn, mah_remain, soc, auto_land_mode
  
    def sendRequestMSP(self, data):
        writeSock(data)

    # def sendMulRequestMSP(self, data, i):
    #     com.writeMulSock(data, i)

    def createPacketMSP(self, msp, payload):
        bf = [ord(char) & 0xFF for char in MSP_HEADER]
        checksum = 0
        pl_size = len(payload) & 0xFF
        bf.append(pl_size)
        checksum ^= pl_size

        bf.append(msp & 0xFF)
        checksum ^= msp & 0xFF

        if payload:
            for k in payload:
                bf.append(k & 0xFF)
                checksum ^= k & 0xFF

        bf.append(checksum & 0xFF)
        return bf

    def sendRequestMSP_SET_RAW_RC(self, channels):
        rc_signals = [0] * 16
        index = 0
        for i in range(8):
            rc_signals[index] = channels[i] & 0xFF
            rc_signals[index + 1] = (channels[i] >> 8) & 0xFF
            index += 2
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_RAW_RC, rc_signals))

    def sendMulRequestMSP_SET_RAW_RC(self, channels):
        rc_signals = [0] * 16
        index = 0
        droneNumber = channels[8]
        for i in range(8):
            rc_signals[index] = channels[i] & 0xFF
            rc_signals[index + 1] = (channels[i] >> 8) & 0xFF
            index += 2
        self.sendMulRequestMSP(self.createPacketMSP(MSP_SET_RAW_RC, rc_signals), droneNumber)

    def sendRequestMSP_SET_POS(self, posArray):
        posData = [0] * 8
        index = 0
        for i in range(4):
            posData[index] = posArray[i] & 0xFF
            posData[index + 1] = (posArray[i] >> 8) & 0xFF
            index += 2
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_POS, posData))

    def sendRequestMSP_SET_COMMAND(self, commandType):
        # print(commandType)
        payload = [commandType & 0xFF]
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_COMMAND, payload))

    def sendRequestMSP_GET_DEBUG(self, requests):
        for req in requests:
            self.sendRequestMSP(self.createPacketMSP(req, []))

    def sendMulRequestMSP_GET_DEBUG(self, requests, index):
        for req in requests:
            self.sendMulRequestMSP(self.createPacketMSP(req, []), index)

    def sendRequestMSP_SET_ACC_TRIM(self, trim_roll, trim_pitch):
        payload = [
            trim_pitch & 0xFF,
            (trim_pitch >> 8) & 0xFF,
            trim_roll & 0xFF,
            (trim_roll >> 8) & 0xFF
        ]
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_ACC_TRIM, payload))

    def sendRequestMSP_ACC_TRIM(self):
        self.sendRequestMSP(self.createPacketMSP(MSP_ACC_TRIM, []))

    def sendRequestMSP_EEPROM_WRITE(self):
        self.sendRequestMSP(self.createPacketMSP(MSP_EEPROM_WRITE, []))

    def sendRequestMSP_ACC_CALIBRATION(self):
        self.sendRequestMSP(self.createPacketMSP(MSP_ACC_CALIBRATION, []))

    def sendRequestMSP_MAG_CALIBRATION(self):
        self.sendRequestMSP(self.createPacketMSP(MSP_MAG_CALIBRATION, []))

    def sendRequestMSP_SET_MOTOR(self, motors):
        # MSP expects 8× uint16 (pad with 1000 for unused slots).
        values = list(motors) + [1000] * (8 - len(motors))
        payload = []
        for v in values[:8]:
            payload.append(v & 0xFF)
            payload.append((v >> 8) & 0xFF)
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_MOTOR, payload))

    def sendRequestMSP_SET_PID(self, pids):
        """Write the 10× MSP PID table.

        MultiWii PID layout (10 triples of [P, I, D] uint8, 30 bytes total):
          0 ROLL  · 1 PITCH · 2 YAW · 3 ALT  · 4 POS · 5 POSR · 6 NAVR
          7 LEVEL · 8 MAG   · 9 VEL

        `pids` is a dict {'roll': {'p':int,'i':int,'d':int}, ...}. Missing axes
        are left at 0/0/0 so you only overwrite what you care about. The
        dashboard currently only exposes roll/pitch/yaw.
        """
        order = ['roll','pitch','yaw','alt','pos','posr','navr','level','mag','vel']
        payload = []
        for axis in order:
            g = pids.get(axis, {})
            payload.append(int(g.get('p', 0)) & 0xFF)
            payload.append(int(g.get('i', 0)) & 0xFF)
            payload.append(int(g.get('d', 0)) & 0xFF)
        self.sendRequestMSP(self.createPacketMSP(MSP_SET_PID, payload))