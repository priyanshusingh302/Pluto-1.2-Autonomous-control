#! python3

import cv2
import numpy as np
import time
from math import atan, cos, sin
import sys
import select
import os
from threading import Thread, Event
from pluto import Pluto

######### This is our Delay Class #####################
# This class create program execution delays
class Delay():

    def __init__(self, delay):

        self.now = 0 # for noting current time
        self.Always_on = False # A flag
        self.initial_time = time.time() # noting initial time
        self.delay = delay # delay in seconds

    def Wait(self): # This function will delay the program execution by @delay seconds everytime it is called

        self.now = time.time() # noting current time
        if (self.now - self.initial_time) >= self.delay: # comparing with initial time for delay
            self.initial_time = self.now # resetting initial time
            return True

    def Wait_Once(self): # this function will delay program execution only Once

        if self.Always_on == False: # if it has not delayed any program execution before
            self.now = time.time() # noting current time
            if (self.now - self.initial_time) >= self.delay: # comparing with initial time for delay
                self.Always_on = True # now it will always be open i.e. it will now never cause program execution delay
                return True
        else: # if it has previously delayed a program execution
            return True # no need to delay anymore on later subsequent function calls

class MyDrone():

    def __init__(self, camera_ID=0):

        # order of convention [roll, pitch, yaw, throttle]
        self.Kp = np.array([3.16227766e+00, -3.16227766e+00, -100, -20]) # Proportional gain
        self.Kd = np.array([7.81569390e+00, -7.81569390e+00, -5, -1])  # Derivative gain
        self.Ki = np.array([0, 0, 0, -5]) # Integral gain

        self.error = np.array([0, 0, 0, 0])
        self.prev_error = np.array([0, 0, 0, 0])
        self.derr = np.array([0, 0, 0, 0])
        self.error_sum = np.array([0, 0, 0, 0])

        self.P_output = np.array([0, 0, 0, 0]) 
        self.D_output = np.array([0, 0, 0, 0])
        self.I_output = np.array([0, 0, 0, 0])

        self.setpoint = np.array([0, 0, 0, 350])
        self.result = np.array([0, 0, 0, 0])
        self.Input = np.array([0, 0, 0, 0])

        self.NEUTRAL = np.array([1500, 1500, 1500, 1500])
        self.HIGH = np.array([1550, 1550, 1550, 1600])
        self.LOW = np.array([1450, 1450, 1450, 1400])

        self.Yaw = 0

        self.measured_position_and_yaw = np.array([0, 0, 0, 0])
        self.estimated_position_and_yaw = np.array([0, 0, 0, 0])

        self.delta_time = 1e-02  # 10 ms

        self.camera_matrix = np.array([[2.36854310e+03, 0.00000000e+00, 8.83847869e+02],
                                       [0.00000000e+00, 2.36416743e+03,
                                           6.26598972e+02],
                                       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.distortion_coefficients = np.array(
            [[-0.53835254, 0.74990547, -0.00897318, 0.00293929, -0.59153803]])

        self.marker_size = 5.9
        self.marker_type = cv2.aruco.DICT_4X4_50
        self.resolution = (1080, 1920)

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.capture = cv2.VideoCapture(camera_ID)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[0])
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[1])
        self.capture.set(cv2.CAP_PROP_FPS, 60)

        self.pluto = Pluto()
        self.pluto.connect()
        self.pluto.ping()

        self.wait = Delay(self.delta_time)

    def __del__(self):
        self.pluto.Land()
        self.pluto.disconnect()
        print("[DISCONNECTED]")
        self.capture.release()
        cv2.destroyAllWindows()
        print("Flight Ended")

    def CalculateYaw(self, vector):

        orientation_vector = vector[0] - vector[2]
        return atan(orientation_vector[0]/orientation_vector[1]), vector[0].astype("uint"), vector[2].astype("uint")

    def Camera_Measurement(self, iD=0):

        ret, frame = self.capture.read()
        if ret:
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

                        self.Yaw, head, tail = self.CalculateYaw(marker_corners[index].reshape(-1, 2))
                        cv2.arrowedLine(frame,tail, head, (255, 0, 0), 2)

                        self.measured_position_and_yaw = np.array(
                            [TransionalVector[index][0][0], TransionalVector[index][0][1], self.Yaw, TransionalVector[index][0][2]])
                else:
                    print("Wrong type for iD")
                    self.__del__()
                    exit(0)
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
            self.__del__()
            exit(0)

    def Transform_coordinates(self, x, y, theta):

        X = (x*cos(theta)) - (y*sin(theta))
        Y = (x*sin(theta)) + (y*cos(theta))
        return X, Y

    def StateEstimator(self):

        self.estimated_position_and_yaw = np.round(self.measured_position_and_yaw)

    def Adjust_PID(self):
        
        for i in range(2):
            if abs(self.error[i]) < 2:
                self.D_output[i] *= 0.01
            if abs(self.error[i]) < 50 and abs(self.derr[i]) > 300:
                self.P_output[i] *= 0.1
                self.D_output[i] *= 10
            if abs(self.error[i]) > 50:
                self.D_output[i] *= 0.0
                if abs(self.derr[i]) > 300:
                    self.P_output[i] *= 0.0

    def controller(self):

        if self.wait.Wait():
            self.error = self.setpoint - self.estimated_position_and_yaw
            self.error[0], self.error[1] = self.Transform_coordinates(self.error[0], self.error[1], self.Yaw)
            self.derr = (self.error - self.prev_error)/self.delta_time
            
            self.P_output = self.Kp * self.error
            self.D_output = self.Kd * self.derr
            self.I_output = self.Ki * self.error_sum

            self.Adjust_PID()

            self.prev_error = self.error
            self.error_sum = self.error_sum + (self.error*self.delta_time)

            self.result = self.P_output + self.D_output + self.I_output

            self.result = self.result + self.NEUTRAL
            self.Input = np.fmin(self.HIGH, np.fmax(self.LOW, self.result))

            # self.Input[0] = int(
            #     min(self.HIGH, max(self.LOW, self.NEUTRAL + self.result[0])))
            # self.Input[1] = int(
            #     min(self.HIGH, max(self.LOW, self.NEUTRAL + self.result[1])))
            # self.Input[2] = int(
            #     min(self.HIGH, max(self.LOW, self.NEUTRAL + self.result[2])))
            # self.Input[3] = int(
            #     min(self.HIGH, max(self.LOW, self.NEUTRAL + self.result[3])))

    def Send_Input(self):

        self.pluto.setRC({"roll":int(self.Input[0]),"pitch":int(self.Input[1]),"yaw":int(self.Input[2]),"throttle":int(self.Input[3])})
        print("input",self.Input.astype("int"))
        # print("setpoint:", self.setpoint)
        print("error",self.error)
        print("derr ", self.derr)
        with open("computation.py", "a") as f:
            f.write(f"error {self.error.astype('int')}")
            f.write(f"derr  {self.derr.astype('int')}\n")
            f.write(f"P     {self.P_output.astype('int')}\n")
            f.write(f"D     {self.D_output.astype('int')}\n")
            f.write(f"I     {self.I_output.astype('int')}\n")
            f.write(f"input {self.Input.astype('int')}\n")
        
            f.write("\n\n")
            f.close()

    def Set_Multiple_Targets(self, targets):

        if isinstance(targets, list):
            self.target_points = targets
        else:
            print("ERROR: targets should be a list")

    def Change_Target(self):

        if hasattr(self, "target_points"):
            if len(self.target_points) > 0:
                self.setpoint = np.array(self.target_points[0])
                self.target_points.pop(0)


if __name__ == "__main__":

    starting_delay = Delay(3)
    p_drone = MyDrone(camera_ID=2)
    p_drone.Set_Multiple_Targets([[70, 0, 0, 350], [-70, 0, 0, 350], [0, 0, 0, 350]])
    while True:
        p_drone.Camera_Measurement()
        if starting_delay.Wait_Once():
            p_drone.StateEstimator()
            p_drone.controller()
            p_drone.Send_Input()
            key = cv2.waitKey(1) & 0xff
            if key  == ord("q"):
                break
            elif key == ord("m"):
                p_drone.pluto.ARM()
            elif key == ord("n"):
                p_drone.pluto.DISARM()
            elif key == ord("c"):
                p_drone.pluto.Land()
            elif key == ord("x"):
                p_drone.pluto.TakeOff()

            elif key==ord("y"):
                p_drone.pluto.AltitudeHold_OFF()
            elif key==ord("u"):
                p_drone.pluto.AltitudeHold_ON()

            elif key==ord("p"):
                p_drone.pluto.HeadFree_OFF()
            elif key==ord("o"):
                p_drone.pluto.HeadFree_ON()

            elif key == ord("s"):
                p_drone.Change_Target()

            elif key == ord("b"):
                
                break

p_drone.__del__()