#! python3

import cv2
import numpy as np
import time
from math import atan, cos, sin
from pluto import Pluto


drone_height = 350 # distance from camera

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
#####################################################

class MyDrone():

    def __init__(self, camera_ID=0, Aruco_ID=0):

        # order of convention [roll, pitch, yaw, throttle]
        self.Kp = np.array([3.16227766e+00, -3.16227766e+00, -10, -10]) # Proportional gain
        self.Kd = np.array([5.81569390e+00, -7.81569390e+00, 0, -1])  # Derivative gain
        self.Ki = np.array([0, 0, 0, 0]) # Integral gain

        self.error = np.array([0, 0, 0, 0]) # Error about each axis (setpoint - actual position)
        self.prev_error = np.array([0, 0, 0, 0]) # previous error about each axis
        self.derr = np.array([0, 0, 0, 0]) # derivative of change of error about each axis
        self.error_sum = np.array([0, 0, 0, 0])

        self.P_output = np.array([0, 0, 0, 0]) 
        self.D_output = np.array([0, 0, 0, 0])
        self.I_output = np.array([0, 0, 0, 0])

        self.setpoint = np.array([0, 0, 0, drone_height])
        self.result = np.array([0, 0, 0, 0])
        self.Input = np.array([0, 0, 0, 0])

        self.x_y_error_tolerance = 10
        self.x_y_derr_tolerance = 500

        self.NEUTRAL = np.array([1500, 1500, 1500, 1500])
        self.HIGH = np.array([1550, 1550, 1600, 1600])
        self.LOW = np.array([1450, 1450, 1400, 1400])

        self.Yaw = 0

        self.measured_position_and_yaw = np.array([0, 0, 0, 0])
        self.estimated_position_and_yaw = np.array([0, 0, 0, 0])

        self.delta_time = 1e-02  # 10 ms
        self.elapsed_time = self.delta_time

        self.camera_matrix = np.array([[2.36854310e+03, 0.00000000e+00, 8.83847869e+02],
                                       [0.00000000e+00, 2.36416743e+03,
                                           6.26598972e+02],
                                       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.distortion_coefficients = np.array(
            [[-0.53835254, 0.74990547, -0.00897318, 0.00293929, -0.59153803]])

        self.marker_size = 5.9
        self.aruco_id = Aruco_ID
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
        self.target_following_delay = Delay(5)
        self.start_pid = False

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

    def Camera_Measurement(self):
        '''
        Function to get the overhead image and compute the orientation and 
        approximate positions of the drone as seen from the overhead camera

        The image from the camera video feed is read and Position approximated through Aruco Marker 
        Detection on the drone. Pose estimation is done with the help of opencv methods.
        '''


        ret, frame = self.capture.read()        # Camera Feed reading frame by frame
        if ret:
            gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(       # Aruco Detection
                gray_img, self.dictionary, parameters=self.parameters)          # marker_corners represent the corner points of the aruco

            if marker_corners:

                cv2.aruco.drawDetectedMarkers(
                    frame, marker_corners, marker_IDs)
                RotationVector, TransionalVector, _ = cv2.aruco.estimatePoseSingleMarkers(              # Orientation calculation using the detected corners
                    marker_corners, self.marker_size, self.camera_matrix, self.distortion_coefficients)
                if isinstance(self.aruco_id, int):
                    L = marker_IDs.flatten().tolist()
                    if self.aruco_id in L:
                        index = L.index(self.aruco_id)
                        cv2.polylines(frame, [marker_corners[index].astype(
                            np.int32)], True, (0, 255, 255), 2)
                        # Drawing the Aruco/Drone axes on the camera feed image
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.distortion_coefficients,
                                          RotationVector[index], TransionalVector[index], 3, 2)
                        cv2.putText(frame, f"x: {round(TransionalVector[index][0][0])} y: {round(TransionalVector[index][0][1])} z: {round(TransionalVector[index][0][2])}", (
                            50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 0, 0), 2)             # Estimated X, Y, Z values in the world coordinates as seen from camera

                        self.Yaw, head, tail = self.CalculateYaw(marker_corners[index].reshape(-1, 2))        # Calculating the Yaw angle
                        cv2.arrowedLine(frame,tail, head, (255, 0, 0), 2)

                        self.measured_position_and_yaw = np.array(                              
                            [TransionalVector[index][0][0], TransionalVector[index][0][1], self.Yaw, TransionalVector[index][0][2]])
                else:           # When the image is not read
                    print("Wrong type for iD")
                    self.__del__()
                    exit(0)         # If camera image could not be accessed, then close the script to avoid random flight (safety)
                h, w = frame.shape[:2]                  # Drawing the extracted imformation on the image
                cv2.circle(
                    frame, (w//2,h//2), 10, (0, 255, 0), 2)
                cv2.circle(
                    frame, (w//2,h//2), 5, (0, 0, 255), -1)
                cv2.line(frame, (w//2 - int(0.05*w), h//2), (w//2 + int(0.05*w), h//2), (0, 255, 0), 2)
                cv2.line(frame, (w//2, h//2 - int(0.05*h)), (w//2, h//2 + int(0.05*h)), (0, 255, 0), 2)
                cv2.imshow("image", frame)

                return True
            else:
                cv2.imshow("image", frame)
                return False

                
        else:
            print("error in openning camera")
            self.__del__()
            exit(0)

    def Transform_coordinates(self, x, y, theta):
        '''
        Converting the extracted camera/World coordinates to the coordinates with respect to drone

        The image is perceived by the camera and the coordinates extracted using image processing will be with respect to the camera,
        but to command the drone , coordinates are required to be converteed to the coordinates with respect to drone. Here the 
        coordinates are rotated to see the axises as seen by drone and not by the camera.
        '''
        X = (x*cos(theta)) - (y*sin(theta))         # Using the rotation matrix to find the transformed coordinates,
        Y = (x*sin(theta)) + (y*cos(theta))         # i.e [X][T] = [X'], X'= Transformed coordinates, X- Original Coordinates, T-rotation matrix 
        return X, Y                                 

    def StateEstimator(self):

        if self.Camera_Measurement():
            self.estimated_position_and_yaw = self.measured_position_and_yaw
            return True
        else:
            return False

    def Adjust_PID(self):
        
        for i in [0, 1, 3]:
            if abs(self.error[i]) < 2:
                self.D_output[i] *= 0.01
            elif abs(self.error[i]) < 50 and abs(self.derr[i]) > 300:
                self.P_output[i] *= 0.1
                # self.D_output[i] *= 10
            elif abs(self.error[i]) > 50:
                self.D_output[i] *= 0.0
                if abs(self.derr[i]) > 300:
                    self.P_output[i] *= 0.0

    def controller(self):

        if not(self.start_pid):
            if (10 <= self.estimated_position_and_yaw[3]) and  (self.estimated_position_and_yaw[3] <= drone_height):
                self.start_pid = True

        if self.wait.Wait():
            if self.StateEstimator():
                # with open("computation.py", "a") as f:
                    self.error = self.setpoint - self.estimated_position_and_yaw
                    self.error[0], self.error[1] = self.Transform_coordinates(self.error[0], self.error[1], self.Yaw)
                    self.error[2] *= 100
                    self.error = np.round(self.error)
                    # f.write(f"error = {self.error}\n")

                    # f.write("self.derr = (self.error - self.prev_error)/self.elapsed_time\n")
                    # f.write(f"self.derr = ({self.error} - {self.prev_error})/{self.elapsed_time}\n")
                    self.derr = (self.error - self.prev_error)/self.elapsed_time
                    # f.write(f"self.derr = {self.derr}\n")
                    
                    # f.write("self.P_output = self.Kp * self.error\n")
                    # f.write(f"{self.P_output} = {self.Kp} * {self.error}\n")
                    self.P_output = self.Kp * self.error
                    # f.write(f"self.P_output = {self.P_output}\n")

                    # f.write("self.D_output = self.Kd * self.derr\n")
                    # f.write(f"{self.D_output} = {self.Kd} * {self.derr}\n")
                    self.D_output = self.Kd * self.derr
                    # f.write(f"self.D_output = {self.D_output}\n")

                    # f.write("self.I_output = self.Kp * self.error_sum\n")
                    # f.write(f"{self.I_output} = {self.Ki} * {self.error_sum}\n")
                    self.I_output = self.Ki * self.error_sum
                    # f.write(f"self.I_output = {self.I_output}\n")

                    self.Adjust_PID()

                    self.prev_error = self.error
                    self.error_sum = self.error_sum + (self.error*self.elapsed_time)

                    self.result = self.P_output + self.D_output + self.I_output

                    self.result = self.result + self.NEUTRAL
                    self.Input = np.fmin(self.HIGH, np.fmax(self.LOW, self.result))

                    # f.write(f"input {self.Input.astype('int')}\n")
                    # f.write("\n\n")
                    # f.close()

                    self.elapsed_time = self.delta_time

                    self.Send_Input()

            else:
                self.elapsed_time += self.delta_time
                self.Send_Input()

    def Send_Input(self):
        '''
        The parameters ( roll, pitch, yaw, throttle ) for the drone as calculated using PID values and the current set point,
        are passed to the drone to make it move.
        '''
        if self.start_pid:
            self.pluto.setRC({"roll":int(self.Input[0]),"pitch":int(self.Input[1]),"yaw":int(self.Input[2]),"throttle":int(self.Input[3])})
        else:
            self.pluto.setRC({"roll":int(1500),"pitch":int(1500),"yaw":int(1500),"throttle":int(1500)})


        print("input",self.Input.astype("int"))
        # print("setpoint:", self.setpoint)
        print("error",self.error)
        print("derr ", self.derr)
        print("flag ", self.setpoint)
        # with open("computation.py", "a") as f:
        #     f.write(f"error {self.error.astype('int')}\n")
        #     f.write(f"derr  {self.derr.astype('int')}\n")
        #     f.write(f"time  {self.elapsed_time}\n")
        #     f.write(f"P     {self.P_output.astype('int')}\n")
        #     f.write(f"D     {self.D_output.astype('int')}\n")
        #     f.write(f"I     {self.I_output.astype('int')}\n")
        #     f.write(f"input {self.Input.astype('int')}\n")
        
        #     f.write("\n\n")
        #     f.close()

    def Set_Multiple_Targets(self, targets):
        '''
        Set the provided target points as the waypoints/set points for the drone path.
        '''
        if isinstance(targets, list):
            self.target_points = targets
        else:
            print("ERROR: targets should be a list")

    def Change_Target(self):
        '''
        If the current setpoint is reached, set the new point from the path as the new setpoint.
        '''
        if hasattr(self, "target_points"):
            if len(self.target_points) > 0 and self.Change_setpoint():
                self.setpoint = np.array(self.target_points[0])
                self.target_points.pop(0)

    def Change_setpoint(self):
        '''
        This function checks if the drone has reached the destined setpoint. If so, it returns true, else False
        '''
        if max(abs(self.setpoint - self.estimated_position_and_yaw)[:2]) <= self.x_y_error_tolerance and max(abs(self.derr[:2])) <= self.x_y_derr_tolerance:
            return True
        else:
            return False


if __name__ == "__main__":

    starting_delay = Delay(2)
    p_drone = MyDrone(camera_ID=2)
    # p_drone.pluto.HeadFree_OFF()
    p_drone.Set_Multiple_Targets([[100, 35, 0, drone_height], [-100, 35, 0, drone_height], [-100, -65, 0, drone_height], [100, -65, 0, drone_height], [100, 35, 0, drone_height], [0, 0, 0,drone_height]])
    while True:
        if starting_delay.Wait_Once():

            p_drone.controller()
            p_drone.Change_Target()


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

            elif key == ord('f'):
                p_drone.start_pid = True

            elif key == ord("b"):
                
                break

p_drone.__del__()
