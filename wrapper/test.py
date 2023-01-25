import cv2
import numpy as np
import time
import sys
import select
import os
from threading import Thread, Event
if os.name == 'nt':
    import msvcrt
    import time
else:
    import tty
    import termios
from pluto import Pluto


class MyDrone():

    def __init__(self, camera_ID=0):

        #  [roll, pitch, yaw, throttle]
        self.Kp = np.array([10, -10, 0, -20])
        self.Kd = np.array([50, -50, 0, -1])
        self.Ki = np.array([20, -20, 0, -5])
        self.error = np.array([0, 0, 0, 0])
        self.prev_error = np.array([0, 0, 0, 0])
        self.error_sum = np.array([0, 0, 0, 0])
        self.setpoint = np.array([0, 0, 0, 300])
        self.Input = np.array([0, 0, 0, 0])
        self.measured_position_and_yaw = np.array([0, 0, 0, 0])
        self.estimated_position_and_yaw = np.array([0, 0, 0, 0])
        self.NEUTRAL = 1500
        self.HIGH = 1600
        self.LOW = 1400
        self.delta_time = 5e-02  # 50 msq
        self.initial_time = 0
        self.now = 0

        self.camera_matrix = np.array([[2.36854310e+03, 0.00000000e+00, 8.83847869e+02],
                                       [0.00000000e+00, 2.36416743e+03,
                                           6.26598972e+02],
                                       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.distortion_coefficients = np.array(
            [[-0.53835254, 0.74990547, -0.00897318, 0.00293929, -0.59153803]])

        self.marker_size = 5.9
        self.marker_type = cv2.aruco.DICT_4X4_50
        self.resolution = (1080, 1920)
        # self.resolution = (720, 1080)
        # self.resolution = (360, 480)

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.capture = cv2.VideoCapture(
            camera_ID, cv2.CAP_DSHOW)  # this is the magic!
        # self.capture.set(cv2.CAP_PROP_SETTINGS, 0)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[0])
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[1])
        self.capture.set(cv2.CAP_PROP_FPS, 60)

        self.pluto = Pluto()
        self.pluto.connect()
        self.pluto.ping()

    def __del__(self):
        self.pluto.Land()
        self.pluto.disconnect()
        self.capture.release()
        cv2.destroyAllWindows()
        print("Flight Ended")

    def getKey():
        if os.name == 'nt':
            timeout = 0.1
            startTime = time.time()
            while (1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - startTime > timeout:
                    return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def CalculateYaw(self, vector):

        return 0

    def Camera_Measurement(self, iD=0):

        ret, frame = self.capture.read()
        if ret:
            # frame = cv2.flip(frame, 1)
            gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
                gray_img, self.dictionary, parameters=self.parameters)

            if marker_corners:

                cv2.aruco.drawDetectedMarkers(
                    frame, marker_corners, marker_IDs)
                RotationVector, TransionalVector, _ = cv2.aruco.estimatePoseSingleMarkers(
                    marker_corners, self.marker_size, self.camera_matrix, self.distortion_coefficients)
                if isinstance(iD, int):
                    L = marker_IDs.flatten().tolist()
                    if iD in L:
                        index = L.index(iD)
                        cv2.polylines(frame, [marker_corners[index].astype(
                            np.int32)], True, (0, 255, 255), 2)
                        # uncomment below line to see the axes
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.distortion_coefficients,
                                          RotationVector[index], TransionalVector[index], 3, 2)
                        cv2.putText(frame, f"x: {round(TransionalVector[index][0][0])} y: {round(TransionalVector[index][0][1])} z: {round(TransionalVector[index][0][2])}", (
                            50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 0, 0), 2)
                        # print(TransionalVector[index][0][0],TransionalVector[index][0][1])
                        self.measured_position_and_yaw = np.array(
                            [TransionalVector[index][0][0], TransionalVector[index][0][1], self.CalculateYaw(TransionalVector), TransionalVector[index][0][2]])
                elif isinstance(iD, list):
                    pass
                else:
                    print("Wrong value for iD")
                    exit(0)
            # frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients, None, self.newcameramtx)
            # frame = frame[self.roi[1]:self.roi[3], self.roi[0]:self.roi[2]]
            h, w = frame.shape[:2]
            cv2.circle(
                frame, (w//2,h//2), 10, (0, 255, 0), 2)
            cv2.circle(
                frame, (w//2,h//2), 5, (0, 0, 255), -1)
            cv2.line(frame, (w//2 - int(0.05*w), h//2), (w//2 + int(0.05*w), h//2), (0, 255, 0), 2)
            cv2.line(frame, (w//2, h//2 - int(0.05*h)), (w//2, h//2 + int(0.05*h)), (0, 255, 0), 2)
            cv2.imshow("image", frame)
        else:
            print("error in openning camera")
            exit(0)

    def StateEstimator(self):

        self.estimated_position_and_yaw = self.measured_position_and_yaw

    def controller(self):

        self.now = time.time()
        if self.now - self.initial_time >= self.delta_time:

            self.error = self.setpoint - self.estimated_position_and_yaw
            self.Input = self.Kp*self.error + self.Kd * \
                (self.error - self.prev_error)/self.delta_time + \
                self.Ki*self.error_sum*self.delta_time
            self.prev_error = self.error
            self.error_sum = self.error_sum + self.error_sum
            self.initial_time = self.now

            self.Input[0] = int(
                min(self.HIGH, max(self.LOW, self.NEUTRAL + self.Input[0])))
            self.Input[1] = int(
                min(self.HIGH, max(self.LOW, self.NEUTRAL + self.Input[1])))
            self.Input[2] = int(
                min(self.HIGH, max(self.LOW, self.NEUTRAL + self.Input[2])))
            self.Input[3] = int(
                min(self.HIGH, max(self.LOW, self.NEUTRAL + self.Input[3])))

    def Send_Input(self):

        self.pluto.setRC({"roll":int(self.Input[0]),"pitch":int(self.Input[1]),"yaw":int(self.Input[2]),"throttle":int(self.Input[3])})
        print(self.Input)


if __name__ == "__main__":

    obj = MyDrone(camera_ID=1)
    while True:
        obj.Camera_Measurement()
        obj.StateEstimator()
        obj.controller()
        obj.Send_Input()
        key = cv2.waitKey(1) & 0xff
        if key  == ord("q"):
            break
        elif key == ord("m"):
            obj.pluto.ARM()
        elif key == ord("n"):
            obj.pluto.DISARM()
        elif key == ord("c"):
            obj.pluto.Land()
        elif key == ord("x"):
            obj.pluto.TakeOff()

        elif key==ord("y"):
            obj.pluto.AltitudeHold_OFF()
        elif key==ord("u"):
            obj.pluto.AltitudeHold_ON()

        elif key==ord("p"):
            obj.pluto.HeadFree_OFF()
        elif key==ord("o"):
            obj.pluto.HeadFree_ON()

        elif key == ord("b"):
            print("[DISCONNECTED]")
            break

obj.__del__()
print("OUT OF LOOP")

