from socket import timeout
import telnetlib
import struct


class Pluto(object):

    __VERSION__ = "1.0"
    __AUTHOR__ = "RoboISM"

    HOST = "192.168.4.1"
    PORT = 23
    MSP = telnetlib.Telnet()

    def setHostPort(self, host, port):
        self.HOST = host
        self.PORT = port

    def connect(self):
        try:
            self.MSP.open(self.HOST, self.PORT, 2)
        except timeout as e:
            print("[ERROR]:", end=' ')
            print(e)

    def disconnect(self):
        self.MSP.close()

    def sendData(self, data):
        try:
            self.MSP.write(data)
        except OSError as e:
            print("[ERROR]:", end=' ')
            print(e)
            return False
        return True

    class MSPCOMMANDS:
        MSP_NULL = 0
        MSP_MODE_RANGES = 34
        MSP_SET_MODE_RANGE = 35
        MSP_ADJUSTMENT_RANGES = 52
        MSP_SET_ADJUSTMENT_RANGE = 53
        MSP_IDENT = 100
        MSP_STATUS = 101
        MSP_RAW_IMU = 102
        MSP_SERVO = 103
        MSP_MOTOR = 104
        MSP_RC = 105
        MSP_RAW_GPS = 106
        MSP_COMP_GPS = 107
        MSP_ATTITUDE = 108
        MSP_ALTITUDE = 109
        MSP_ANALOG = 110
        MSP_BOX = 113
        MSP_MISC = 114
        MSP_BOXNAMES = 116
        MSP_BOXIDS = 119
        MSP_SET_RAW_RC = 200
        MSP_ACC_CALIBRATION = 205
        MSP_MAG_CALIBRATION = 206
        MSP_SET_MISC = 207
        MSP_SET_HEAD = 211
        MSP_SET_COMMAND = 217

    def toInt16(self, data):
        if (len(data) == 2):
            return struct.unpack("@h", struct.pack("<BB", data[0], data[1]))[0]
        else:
            return None

    def toUInt16(self, data):
        if (len(data) == 2):
            return struct.unpack("@H", struct.pack("<BB", data[0], data[1]))[0]
        else:
            return None

    def toInt32(self, data):
        if (len(data) == 4):
            return struct.unpack("@i", struct.pack("<BBBB", data[0], data[1], data[2], data[3]))[0]
        else:
            return None

    def toUInt32(self, data):
        if (len(data) == 4):
            return struct.unpack("@I", struct.pack("<BBBB", data[0], data[1], data[2], data[3]))[0]
        else:
            return None

    def fromInt16(self, value):
        return struct.unpack("<BB", struct.pack("@h", value))

    def fromUInt16(self, value):
        return struct.unpack("<BB", struct.pack("@H", value))

    def fromInt32(self, value):
        return struct.unpack("<BBBB", struct.pack("@i", value))

    def fromUInt32(self, value):
        return struct.unpack("<BBBB", struct.pack("@I", value))

    def encodePacket(self, command, data=None):
        if (data is None):
            dataSize = 0
        else:
            if len(data) < 256:
                dataSize = len(data)
            else:
                return False
        packet = bytearray()
        packet.append(ord('$'))
        packet.append(ord('M'))
        packet.append(ord('<'))
        packet.append(dataSize)
        checksum = dataSize
        packet.append(command)
        checksum = (checksum ^ command)
        if (dataSize > 0):
            for b in data:
                packet.append(b)
                checksum = (checksum ^ b)

        packet.append(checksum)
        return self.sendData(packet)

    def decodePacket(self, command, packet):
        len = int.from_bytes(packet[3])
        if (int.from_bytes(packet[4]) == command):
            return packet[5:5+len]
        return None

    def getData(self, command, expexted_length):
        data = []
        self.encodePacket(command)
        conn = self.MSP.get_socket()
        if (conn.recv(2) == b'$M'):
            conn.recv(1)
            leng = int.from_bytes(conn.recv(1))
            if (command == int.from_bytes(conn.recv(1)) and leng == expexted_length):
                checksum = leng ^ command
                for i in range(0, leng):
                    k = int.from_bytes(conn.recv(1))
                    data.append(k)
                    checksum = checksum ^ k

                # if (checksum == conn.recv(1)):
                #     return data
                # else:
                #     return None
                print(data)
                return data
            else:
                return None
        else:
            return None

    def setRC(self, values):
        data = bytearray()
        throttle = 0
        pitch = 0
        yaw = 0
        roll = 0
        aux = [0, 0, 0, 0]
        if isinstance(values, dict):
            if ("pitch" in values):
                pitch = values["pitch"]
            if ("roll" in values):
                roll = values["roll"]
            if ("yaw" in values):
                yaw = values["yaw"]
            if ("throttle" in values):
                throttle = values["throttle"]
            for i in range(1, 5):
                if ("aux" + str(i) in values):
                    aux[i-1] = values["aux" + str(i)]
            r = self.fromUInt16(roll)
            data.append(r[0])
            data.append(r[1])
            r = self.fromUInt16(pitch)
            data.append(r[0])
            data.append(r[1])
            r = self.fromUInt16(yaw)
            data.append(r[0])
            data.append(r[1])
            r = self.fromUInt16(throttle)
            data.append(r[0])
            data.append(r[1])
            for i in range(0, 4):
                r = self.fromUInt16(aux[i])
                data.append(r[0])
                data.append(r[1])
            return self.encodePacket(self.MSPCOMMANDS.MSP_SET_RAW_RC, data)
        else:
            return False

    def setCommand(self, value):
        data = bytearray()
        com = 0
        if isinstance(value, int):
            if (value > 0):
                com = value
            r = self.fromUInt16(com)
            data.append(r[0])
            data.append(r[1])

            return self.encodePacket(self.MSPCOMMANDS.MSP_SET_COMMAND, data)
        else:
            return False

    def temp_disconnect(self):
        data = bytearray()
        data.append(ord('!'))
        data.append(ord('D'))
        data.append(ord('I'))
        data.append(ord('S'))
        self.encodePacket(0, data)

    def setThrottle(self, value):
        self.setRC({"throttle": value})

    def setPitch(self, value):
        self.setRC({"pitch": value})

    def setRoll(self, value):
        self.setRC({"roll": value})

    def setYaw(self, value):
        self.setRC({"yaw": value})

    def ARM(self):
        self.setRC({"aux4": 1500})

    def DISARM(self):
        self.setRC({"aux4": 1000})

    def TakeOff(self):
        self.setCommand(1)

    def Land(self):
        self.setCommand(2)

    def BackFlip(self):
        self.setCommand(3)

    def FrontFlip(self):
        self.setCommand(4)

    def RightFlip(self):
        self.setCommand(5)

    def LeftFlip(self):
        self.setCommand(6)

    def getAltitude(self):
        data = self.getData(self.MSPCOMMANDS.MSP_ALTITUDE, 6)
        if (data):
            return self.toInt32(data[0:4])
        else:
            return None

    def getVariometer(self):
        data = self.getData(self.MSPCOMMANDS.MSP_ALTITUDE, 6)
        if (data):
            return self.toInt16(data[4:6])
        else:
            return None
