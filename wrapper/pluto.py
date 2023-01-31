from socket import timeout
from threading import Thread, Event
from time import sleep, time
import telnetlib
import struct
import sys


class Pluto(object):

    __VERSION__ = "1.0"
    __AUTHOR__ = "RoboISM"

    HOST = "192.168.4.1"
    PORT = 23
    MSP = telnetlib.Telnet()
    soc = None

    #Creating a separate thread to monitor the packets that are being recieved from the pluto drone ...............
    def __init__(self):
        self.monitorThread = Thread(target=self.monitorSerialPort)
        self.exitNow = Event()
        self.responses = {}
        self.responseTimeout = 3
    #..............................................................................................................

    #Destructor to close the monitor thread when we want to stop the communication...................................
    def __del__(self):
        if self.monitorThread.is_alive():
            self.exitNow.set()
    #.............................................................................................................

    #It can be used to change the host and port
    def setHostPort(self, host, port):
        self.HOST = host
        self.PORT = port

    #Function to connect the Pluto drone with laptop and at the same time start the monitor thread to recieve the packets sent by pluto drone 
    def connect(self):
        try:
            self.MSP.open(self.HOST, self.PORT, 2)
            self.soc = self.MSP.get_socket()
            self.monitorThread.start()
        except timeout as e:
            print("[ERROR]:", end=' ')
            print(f'{e} \n[EXITING]')
            sys.exit()
    #............................................................................................................
    
    #To disconnect the drone.....................................................................................
    def disconnect(self):
        if self.monitorThread.is_alive():
            self.exitNow.set()
        self.MSP.close()
    #............................................................................................................


    class MSPResponse:
        def __init__(self):
            self.finished = False
            self.data = []

    class MSPSTATES:
        IDLE = 0
        HEADER_START = 1
        HEADER_M = 2
        HEADER_ARROW = 3
        HEADER_SIZE = 4
        HEADER_CMD = 5

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

    #To convert the binary data into 
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

    def monitorSerialPort(self):
        print("[LISTENING]")
        state = self.MSPSTATES.IDLE
        data = bytearray()
        dataSize = 0
        dataChecksum = 0
        command = self.MSPCOMMANDS.MSP_NULL
        inByte = None
        while (not self.exitNow.isSet()):
            try:
                inByte = ord(self.soc.recv(1))
            except:
                pass
            # print(inByte)
            if (inByte != None):
                if (state == self.MSPSTATES.IDLE):
                    state = self.MSPSTATES.HEADER_START if (
                        inByte == 36) else self.MSPSTATES.IDLE  # chr(36)=='$'
                elif (state == self.MSPSTATES.HEADER_START):
                    state = self.MSPSTATES.HEADER_M if (
                        inByte == 77) else self.MSPSTATES.IDLE  # chr(77)=='M'
                elif (state == self.MSPSTATES.HEADER_M):
                    state = self.MSPSTATES.HEADER_ARROW if (
                        inByte == 62) else self.MSPSTATES.IDLE  # chr(62)=='>'
                elif (state == self.MSPSTATES.HEADER_ARROW):
                    dataSize = inByte
                    data = bytearray()
                    dataChecksum = inByte
                    state = self.MSPSTATES.HEADER_SIZE
                elif (state == self.MSPSTATES.HEADER_SIZE):
                    command = inByte
                    dataChecksum = (dataChecksum ^ inByte)
                    state = self.MSPSTATES.HEADER_CMD
                elif (state == self.MSPSTATES.HEADER_CMD) and (len(data) < dataSize):
                    data.append(inByte)
                    dataChecksum = (dataChecksum ^ inByte)
                elif (state == self.MSPSTATES.HEADER_CMD) and (len(data) >= dataSize):
                    if (dataChecksum == inByte):  # If checksum matches
                        self.responses[command].finished = True
                        self.responses[command].data = data
                    else:
                        self.responses[command].finished = True
                        self.responses[command].data = None
                    state = self.MSPSTATES.IDLE
            else:
                sleep(0)
        self.disconnect()


    def sendData(self, data):
        try:
            self.MSP.write(data)
        except AttributeError as e:
            print("[ERROR]:", end=' ')
            print(e)
            return False
        except OSError as e:
            print("[ERROR]:", end=' ')
            print(e)
            return False
        return True


    def waitForResponse(self, command):
        if ((command) in self.responses):
            startTime = time()
            while True:
                if self.responses[command].finished:
                    return self.responses[command].data
                if (time() - startTime > self.responseTimeout):
                    return False
                sleep(0)
        else:
            return False
    #............................................................................................


    #This function takes the data to be sent to the drone and convert it into the required format
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
        self.responses.update({command: self.MSPResponse()})
        return self.sendData(packet)
    #.............................................................................................


    def getData(self, command):
        self.encodePacket(command)
        return self.waitForResponse(command)

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
            r = self.fromUInt16(throttle)
            data.append(r[0])
            data.append(r[1])
            r = self.fromUInt16(yaw)
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
        '''For testing and disconnecting from virtual server'''
        data = bytearray()
        data.append(ord('!'))
        data.append(ord('D'))
        data.append(ord('I'))
        data.append(ord('S'))
        self.encodePacket(0, data)
        if self.monitorThread.is_alive():
            self.exitNow.set()

    def setThrottle(self, value):
        '''SET Throttle value  UNIT:PWM   RANGE:900-2100'''
        self.setRC({"throttle": value})

    def setPitch(self, value):
        '''SET Pitch value  UNIT:PWM   RANGE:900-2100'''
        self.setRC({"pitch": value})

    def setRoll(self, value):
        '''SET Roll value  UNIT:PWM   RANGE:900-2100'''
        self.setRC({"roll": value})

    def setYaw(self, value):
        '''SET Yaw value  UNIT:PWM   RANGE:900-2100'''
        self.setRC({"yaw": value})

    def ARM(self):
        '''ARM Pluto'''
        self.setRC({"aux4": 1500})

    def DISARM(self):
        '''DISARM Pluto'''
        self.setRC({"aux4": 1000})

    def AltitudeHold_ON(self):
        '''Toggles Altitude Hold Mode ON'''
        self.setRC({"aux3": 1500})

    def AltitudeHold_OFF(self):
        '''Toggles Altitude Hold Mode OFF'''
        self.setRC({"aux3": 1000})

    def DevMode_ON(self):
        '''Toggles Developer Mode ON'''
        self.setRC({"aux2": 1500})

    def DevMode_OFF(self):
        '''Toggles Developer Mode OFF'''
        self.setRC({"aux2": 1000})

    def HeadFree_ON(self):
        '''Toggles Headfree Mode ON'''
        self.setRC({"aux1": 1500})

    def HeadFree_OFF(self):
        '''Toggles Headfree Mode OFF'''
        self.setRC({"aux1": 1000})

    def TakeOff(self):
        '''Command for pluto to Take off'''
        self.setCommand(1)

    def Land(self):
        '''Command for pluto to Land'''
        self.setCommand(2)

    def BackFlip(self):
        '''Command for pluto to do backflip'''
        self.setCommand(3)

    def FrontFlip(self):
        '''Command for pluto to do frontflip'''
        self.setCommand(4)

    def RightFlip(self):
        '''Command for pluto to do rightflip'''
        self.setCommand(5)

    def LeftFlip(self):
        '''Command for pluto to do leftflip'''
        self.setCommand(6)

    def getAltitude(self):
        '''Returns estimated altitude. Units: Centimeters'''
        data = self.getData(self.MSPCOMMANDS.MSP_ALTITUDE)
        if (data):
            return self.toInt32(data[0:4])
        else:
            return None

    def getVariometer(self):
        '''Returns estimated vertical velocity. Units: cm/s'''
        data = self.getData(self.MSPCOMMANDS.MSP_ALTITUDE)
        if (data):
            return self.toInt16(data[4:6])
        else:
            return None

    def getAcc(self):
        '''Returns RAW data from accelerometer [X-axis, Y-axis, Z-axis]'''
        data = self.getData(self.MSPCOMMANDS.MSP_RAW_IMU)
        if (data):
            return [self.toInt16(data[0:2]), self.toInt16(data[2:4]), self.toInt16(data[4:6])]
        else:
            return None

    def getGyro(self):
        '''Returns RAW data from gyroscope [X-axis, Y-axis, Z-axis]'''
        data = self.getData(self.MSPCOMMANDS.MSP_RAW_IMU)
        if (data):
            return [self.toInt16(data[6:8]), self.toInt16(data[8:10]), self.toInt16(data[10:12])]
        else:
            return None

    def getMag(self):
        '''Returns RAW data from magnetometer [X-axis, Y-axis, Z-axis]'''
        data = self.getData(self.MSPCOMMANDS.MSP_RAW_IMU)
        if (data):
            return [self.toInt16(data[12:14]), self.toInt16(data[14:16]), self.toInt16(data[16:18])]
        else:
            return None

    def getRoll(self):
        '''Returns Roll. Units : Decidegrees'''
        data = self.getData(self.MSPCOMMANDS.MSP_ATTITUDE)
        if (data):
            return self.toInt16(data[0:2])
        else:
            return None

    def getPitch(self):
        '''Returns pitch. Units : Decidegrees'''
        data = self.getData(self.MSPCOMMANDS.MSP_ATTITUDE)
        if (data):
            return self.toInt16(data[2:4])
        else:
            return None

    def getYaw(self):
        '''Returns Yaw. Units : Decidegrees'''
        data = self.getData(self.MSPCOMMANDS.MSP_ATTITUDE)
        if (data):
            return self.toInt16(data[4:6])
        else:
            return None


    def ping(self):
        tim=time()
        x = self.getAltitude()
        print(f'PING:{round((time()-tim)*1000.0, 2)} ms')


if '__name__' == '__main__':
    Pluto()        
