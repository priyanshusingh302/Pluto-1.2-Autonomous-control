from socket import timeout
from threading import Thread, Event
from time import sleep, time
import struct
import sys
import serial

class Pluto_Serial(object):

    __VERSION__ = "1.0"

    HOST = "192.168.4.1"        # default HOST
    PORT = 23                   # default PORT
    ser = serial.Serial()
    BAUDRATE = 115200
    Serial_PORT = 'COM5'

    def __init__(self):
        '''constructor'''
        self.monitorThread = Thread(target=self.monitorSerialPort)  # a thread instance that runs the monitorSerialPort function when started
        self.exitNow = Event()                                      # an event object used to indicate when the monitorSerialPort function should stop listening
        self.responses = {}                                         # a dictionary to store the responses received from the serial port
        self.responseTimeout = 3                                    # a value that specifies the timeout duration for the responses.
        self.ser.baudrate=self.BAUDRATE
        self.ser.port = self.Serial_PORT

    def __del__(self):
        '''destructor'''
        if self.monitorThread.is_alive():
            self.exitNow.set()

    def setHostPort(self, host, port):
        '''This function sets the host and port of the Pluto device'''
        self.HOST = host
        self.PORT = port

    def setCOM(self,serial_port=str):
        self.ser.port=serial_port

    def connect(self):
        '''Function to connect the Pluto drone with laptop and at the same time start the monitor thread to recieve 
        the packets sent by pluto drone '''
        try:
            self.ser.open()
            self.monitorThread.start()                  # starts the thread
        except timeout as e:
            print("[ERROR]:", end=' ')
            print(f'{e} \n[EXITING]')
            sys.exit()
    
    def disconnect(self):
        '''disconnects from pluto'''
        if self.monitorThread.is_alive():
            self.exitNow.set()
        self.ser.close()


    class MSPResponse:                      # This class holds the information about a response received
        def __init__(self):
            self.finished = False           # a boolean indicating whether the response has finished or not.
            self.data = []                  # a list of data received from the response.

    class MSPSTATES:
        '''
        It specifies the values that represent different states of the MSP protocol

        IDLE: The protocol is in an idle state
        HEADER_START: The start of a header has been received
        HEADER_M: The "M" character has been received
        HEADER_ARROW: The ">" character has been received
        HEADER_SIZE: The size of the data packet has been received
        HEADER_CMD: The command of the data packet has been received
        '''
        IDLE         = 0
        HEADER_START = 1
        HEADER_M     = 2
        HEADER_ARROW = 3
        HEADER_SIZE  = 4
        HEADER_CMD   = 5

    class MSPCOMMANDS:
        '''
        This is a Python class that defines constants for different MultiWii Serial Protocol (MSP) commands. 
        The different commands can be used to request data from the flight controller or to set different parameters.
        '''
        MSP_NULL                    = 0
        MSP_MODE_RANGES             = 34
        MSP_SET_MODE_RANGE          = 35
        MSP_ADJUSTMENT_RANGES       = 52
        MSP_SET_ADJUSTMENT_RANGE    = 53
        MSP_IDENT                   = 100
        MSP_STATUS                  = 101
        MSP_RAW_IMU                 = 102
        MSP_SERVO                   = 103
        MSP_MOTOR                   = 104
        MSP_RC                      = 105
        MSP_RAW_GPS                 = 106
        MSP_COMP_GPS                = 107
        MSP_ATTITUDE                = 108
        MSP_ALTITUDE                = 109
        MSP_ANALOG                  = 110
        MSP_BOX                     = 113
        MSP_MISC                    = 114
        MSP_BOXNAMES                = 116
        MSP_BOXIDS                  = 119
        MSP_SET_RAW_RC              = 200
        MSP_ACC_CALIBRATION         = 205
        MSP_MAG_CALIBRATION         = 206
        MSP_SET_MISC                = 207
        MSP_SET_HEAD                = 211
        MSP_SET_COMMAND             = 217

    '''Converts data to requred format'''
    def toInt16(self, data):
        ''' from 2 bytes to INT16'''
        if (len(data) == 2):
            return struct.unpack("@h", struct.pack("<BB", data[0], data[1]))[0]
        else:
            return None


    def toUInt16(self, data):
        '''from 2 bytes to UINT16'''
        if (len(data) == 2):
            return struct.unpack("@H", struct.pack("<BB", data[0], data[1]))[0]
        else:
            return None

    def toInt32(self, data):
        '''from 4 bytes to INT32'''
        if (len(data) == 4):
            return struct.unpack("@i", struct.pack("<BBBB", data[0], data[1], data[2], data[3]))[0]
        else:
            return None

    def toUInt32(self, data):
        '''from 4 bytes to UINT32'''
        if (len(data) == 4):
            return struct.unpack("@I", struct.pack("<BBBB", data[0], data[1], data[2], data[3]))[0]
        else:
            return None

    def fromInt16(self, value):
        '''from INT16 to 2 bytes'''
        return struct.unpack("<BB", struct.pack("@h", value))

    def fromUInt16(self, value):
        '''from UINT16 to 2 bytes'''
        return struct.unpack("<BB", struct.pack("@H", value))

    def fromInt32(self, value):
        '''from INT32 to 4 bytes'''
        return struct.unpack("<BBBB", struct.pack("@i", value))

    def fromUInt32(self, value):
        '''from UINT32 to 4 bytes'''
        return struct.unpack("<BBBB", struct.pack("@I", value))

    def monitorSerialPort(self):

        '''
        This code listens to incoming data from Pluto. The code uses a state machine to parse the incoming 
        data stream, which consists of messages starting with "$M>" header, followed by the message size, command, 
        and data payload. The data payload is also followed by a checksum value.
        '''
        
        print("[LISTENING]")
        state = self.MSPSTATES.IDLE                                             # sets state to IDLE
        data = bytearray()                                                      # creates a byte array
        dataSize = 0
        dataChecksum = 0
        command = self.MSPCOMMANDS.MSP_NULL
        inByte = None
        while (not self.exitNow.isSet()):
            try:
                inByte = ord(self.ser.read(1))                                  # tries to receive 1 byte of data
            except:
                pass
            if (inByte != None):        
                if (state == self.MSPSTATES.IDLE):             
                    state = self.MSPSTATES.HEADER_START if (
                        inByte == 36) else self.MSPSTATES.IDLE                  # chr(36)=='$'
                
                elif (state == self.MSPSTATES.HEADER_START):
                    state = self.MSPSTATES.HEADER_M if (
                        inByte == 77) else self.MSPSTATES.IDLE                  # chr(77)=='M'
               
                elif (state == self.MSPSTATES.HEADER_M):
                    state = self.MSPSTATES.HEADER_ARROW if (
                        inByte == 62) else self.MSPSTATES.IDLE                  # chr(62)=='>'
                
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
                    if (dataChecksum == inByte):                                # if checksum matches
                        self.responses[command].finished = True                 # sets the finished flag to true
                        self.responses[command].data = data                     # saves the data received
                    else:                                                       # else
                        self.responses[command].finished = True                 # sets the finished flag to true
                        self.responses[command].data = None                     # discards the data
                    state = self.MSPSTATES.IDLE
            else:
                sleep(0)                                                        # sleeps the thread if no byte is received

        self.disconnect()

    def sendData(self, data):
        '''function to send a packet using Telnet'''
        try:
            self.ser.write(data)
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
        '''this function waits until the requested data is recieved and then return the data'''
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


    def encodePacket(self, command, data=None):
        '''
        This function creates and sends a packet of MSP data to the serial port. The packet contains a header 
        with information about the data, followed by the data itself, and finally a checksum that is used to check 
        the integrity of the data during transmission.
        '''
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

    def getData(self, command):
        '''this function encapsulates sending the get command and returning the data when it is recieved'''
        self.encodePacket(command)
        return self.waitForResponse(command)

    def setRC(self, values):
        '''
        Sends RC values to drone
        Format: Dictionary with following key and value pair
        {
            "roll":Value,
            "pitch":Value,
            "yaw":Value,
            "throttle":Value,
            "aux1":Value,
            "aux2":Value,
            "aux3":Value,
            "aux4":Value,
        }
        '''
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
        '''
        sends diffrent commands to drone to do certain action
        1 : Take-off
        2 : Land
        3 : Back Flip
        4 : Front Flip
        5 : Right Flip
        6 : Left Flip
        '''
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
        '''Returns the round trip time of sending a command and receiving it.  Unit : ms'''
        tim=time()
        x = self.getAltitude()
        print(f'PING:{round((time()-tim)*1000.0, 2)} ms')


if '__name__' == '__main__':
    Pluto_Serial()        