#! python3

# ---- importing required libraries ----
import cv2
import numpy as np
from time import time
from math import atan, cos, sin
from pluto import Pluto

drone_height = 350     # distance from camera

''' This is our Delay Class 
This class creates program execution delays '''

class Delay():

    def __init__(self, delay):
        self.now          = 0                                # for noting current time
        self.Always_on    = False                            # A flag 
        self.initial_time = time()                           # noting initial time
        self.delay        = delay                            # delay in seconds

    def Wait(self): 

        ''' This function will delay the program execution by @delay seconds everytime it is called '''
        self.now = time()                                    # noting current time
        if (self.now - self.initial_time) >= self.delay:     # comparing with initial time for delay
            self.initial_time = self.now                     # resetting initial time
            return True

    def Wait_Once(self):

        '''This function will delay program execution only once '''
        if not self.Always_on :                              # if it has not delayed any program execution before
            self.now = time.time()                           # noting current time
            if (self.now - self.initial_time) >= self.delay: # comparing with initial time for delay
                self.Always_on = True                        # now it will always be open i.e. it will now never cause program execution delay
                return True
        else:                                                # if it has previously delayed a program execution
            return True                                      # no need to delay anymore on later subsequent function calls

class MyDrone():

    def __init__(self, camera_ID=0, camera_resolution=(1080, 1920), Aruco_ID=0, marker_type="DICT_4X4_50"):

        self.PID_initialize()
        self.ArucoAndCamera_initialize(camera_ID, camera_resolution, Aruco_ID, marker_type)

        #------- starting connection with pluto ------
        self.pluto = Pluto()                       
        self.pluto.connect()                       
        self.pluto.ping()                          
       
        self.wait      = Delay(self.delta_time)                         # this will be used for sampling frequency of pid
        self.start_pid = False                                          # this flag will be used to start pid algorithm

        self.minimum_frames_for_hovering     = 20
        self.minimum_frames_for_not_hovering = 100
        self.frame_for_hovering              = 0
        self.frame_for_not_hovering          = 0


    def PID_initialize(self):
        # order of convention [roll, pitch, yaw, throttle]
        self.Kp = np.array([3.16227766e+00, -3.16227766e+00, -10, -10]) # Proportional gain
        self.Kd = np.array([7.81569390e+00, -7.81569390e+00, 0, -1])    # Derivative gain
        self.Ki = np.array([0, 0, 0, 0])                                # Integral gain

        self.error      = np.array([0, 0, 0, 0])                        # Error about roll, pitch, yaw and throttle (setpoint - actual position)
        self.prev_error = np.array([0, 0, 0, 0])                        # previous error about roll, pitch, yaw and throttle
        self.derr       = np.array([0, 0, 0, 0])                        # derivative of change of error about roll, pitch, yaw and throttle
        self.error_sum  = np.array([0, 0, 0, 0])                        # sum of errors about roll, pitch, yaw and throttle

        self.P_output   = np.array([0, 0, 0, 0])                        # stores Proportional output of each axis
        self.D_output   = np.array([0, 0, 0, 0])                        # stores Derivative output of each axis
        self.I_output   = np.array([0, 0, 0, 0])                        # stores Integral output of each axis

        self.result     = np.array([0, 0, 0, 0])                        # to store the PID output of roll, pitch, yaw and throttle
        self.Input      = np.array([0, 0, 0, 0])                        # it will store the value that will be ultimately send to the drone

        self.setpoint   = np.array([0, 0, 0, drone_height])             # initial setpoint

        self.x_y_error_tolerance = 10                                   # +- tolerance value of error for x and y directions
        self.x_y_derr_tolerance  = 500                                  # +- tolerance value of derr for x and y directions

        self.NEUTRAL    = np.array([1500, 1500, 1500, 1500])            # Neutral value for roll, pitch, yaw and throttle
        self.HIGH       = np.array([1550, 1550, 1600, 1600])            # upperbound value for roll, pitch, yaw and throttle
        self.LOW        = np.array([1450, 1450, 1400, 1400])            # lowerbound value for roll, pitch, yaw and throttle

        self.Yaw = 0                                                    # initial yaw

        self.measured_position_and_yaw  = np.array([0, 0, 0, 0])        # it will store the measured x, y, z position and yaw of drone
        self.estimated_position_and_yaw = np.array([0, 0, 0, 0])        # it will store the estimated x, y, z position and yaw of drone

        self.delta_time   = 1e-02                                       # 10 ms, the time interval of running pid
        self.elapsed_time = self.delta_time                             # this time will be used to calculate derivative of error

    def ArucoAndCamera_initialize(self,camera_ID=0, camera_resolution=(1080, 1920), Aruco_ID=0, marker_type="DICT_4X4_50"): 

        # --------------  Camera calliberation Params --------------------
        self.camera_matrix = np.array([[2.36854310e+03, 0.00000000e+00, 8.83847869e+02],                  
                                       [0.00000000e+00, 2.36416743e+03,                                   
                                           6.26598972e+02],                                               
                                       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])                 
        self.distortion_coefficients = np.array(                                                          
            [[-0.53835254, 0.74990547, -0.00897318, 0.00293929, -0.59153803]])                            
        
        # -------------- Aruco Params -----------------------------
        self.marker_size = 5.9                                                       # side length of the aruco marker used 
        self.aruco_id    = Aruco_ID                                                  # aruco id used in drone
        self.marker_type = getattr(cv2.aruco, f"{marker_type}")                      # getting marker type
        self.resolution  = camera_resolution                                         # resolution of camera

        self.dictionary  = cv2.aruco.getPredefinedDictionary(self.marker_type)       # generating aruco dictionary
        self.parameters  = cv2.aruco.DetectorParameters()                            # getting aruco parameters
        detector         = cv2.aruco.ArucoDetector(self.dictionary, self.parameters) # creating detector

        #---------------- initializing camera ---------------------
        self.capture = cv2.VideoCapture(camera_ID)                                   # starting camera                                       
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[0])              # setting camera frame height      
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[1])               # setting camera frame height       
        self.capture.set(cv2.CAP_PROP_FPS, 60)                                       # setting camera fps                                        

    def __del__(self):
        self.pluto.Land()                                                            # land the drone
        self.pluto.disconnect()                                                      # break connection
        print("[DISCONNECTED]")
        self.capture.release()                                                       # release camera object
        cv2.destroyAllWindows()                                                      # close all open image videos
        print("Flight Ended")

    def CalculateYaw(self, vector):                
        ''' This function calculates yaw of the drone with respect to the camera frame'''
        orientation_vector = vector[0] - vector[2]                                                                   # orientation vector of drone
        return atan(orientation_vector[0]/orientation_vector[1]), vector[0].astype("uint"), vector[2].astype("uint") # returning inclination of this vector

    def Camera_Measurement(self):
        '''
        Function to get the overhead image and compute the orientation and 
        approximate positions of the drone as seen from the overhead camera.
        The image from the camera video feed is read and position is approximated through Aruco Marker 
        Detection.Pose estimation is done with the help of opencv methods.
        '''
        ret, frame = self.capture.read()                                             # Camera Feed reading frame by frame
        if ret:                                                                      # if image is captured
            gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)                       # converting to gray scale
            marker_corners, marker_IDs, _ = cv2.aruco.detectMarkers(                 # Aruco Detection
                gray_img, self.dictionary, parameters=self.parameters)               # marker_corners represent the corner points of the aruco

            if marker_corners:                                                       # if marker corners are detected i.e. any aruco is detected
                cv2.aruco.drawDetectedMarkers(                                       # Drawing detected aruco markers
                    frame, marker_corners, marker_IDs, borderColor=(0, 255, 255))
                _, TransionalVector, _ = cv2.aruco.estimatePoseSingleMarkers(        # position calculation using the aruco
                    marker_corners, self.marker_size, self.camera_matrix, self.distortion_coefficients)
                if isinstance(self.aruco_id, int):    
                    L = marker_IDs.flatten().tolist()
                    if self.aruco_id in L:
                        frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients)
                        index = L.index(self.aruco_id)
                        ''' uncomment below line to see the aruco border in bold
                        (the border may appear to be shifted near frame boundaries since we are performing undistortion operation on image)'''
                        # cv2.polylines(frame, [marker_corners[index].astype(
                        #     np.int32)], True, (0, 255, 255), 2)
                        '''uncomment below line to see the axes
                        (the axes may appear to be shifted near frame boundaries since we are performing undistortion operation on image) '''
                        # cv2.drawFrameAxes(frame, self.camera_matrix, self.distortion_coefficients,
                        #                   RotationVector[index], TransionalVector[index], 3, 2)

                        cv2.putText(frame, f"x: {round(TransionalVector[index][0][0])} y: {round(TransionalVector[index][0][1])} z: {round(TransionalVector[index][0][2])}", (
                            50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 0, 0), 2)

                        self.Yaw, head, tail = self.CalculateYaw(marker_corners[index].reshape(-1, 2))       # Calculating the Yaw angle
                        '''uncomment below line to see yaw vector
                        (the vector may appear to be shifted near frame boundaries since we are performing undistortion operation on image)'''
                        # cv2.arrowedLine(frame,tail, head, (255, 0, 0), 2)                                  # drawing yaw heading

                        self.measured_position_and_yaw = np.array(
                            [TransionalVector[index][0][0], TransionalVector[index][0][1], self.Yaw, TransionalVector[index][0][2]])
                else:
                    print("Wrong type for iD") # else wrong type of aruco id
                    self.__del__()
                    exit(0)

                # -------------------for drawing on image frame ---------------------------------
                # h, w = frame.shape[:2]                                                                      
                # cv2.circle(                                                                                 
                #     frame, (w//2,h//2), 10, (0, 255, 0), 2)                                                 
                # cv2.circle(                                                                                 
                #     frame, (w//2,h//2), 5, (0, 0, 255), -1)                                                 
                # cv2.line(frame, (w//2 - int(0.05*w), h//2), (w//2 + int(0.05*w), h//2), (0, 255, 0), 2)     
                # cv2.line(frame, (w//2, h//2 - int(0.05*h)), (w//2, h//2 + int(0.05*h)), (0, 255, 0), 2)     
                
                cv2.imshow("image", frame) # showing image frame
                return True                # return true if aruco detected

            else:
                frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients)
                cv2.imshow("image", frame)
                return False # if aruco not detected return false

                
        else: # camera not found
            print("error in openning camera")
            self.__del__()
            exit(0)

    def Transform_coordinates(self, x, y, theta):
        '''
        Converting the extracted camera/World coordinates to the coordinates with respect to drone
        The image is perceived by the camera and the coordinates extracted using image processing will be with respect to the camera,
        but to command the drone , coordinates are required to be converteed to the coordinates with respect to drone. Here the 
        coordinates are rotated to see the axes as seen by drone and not by the camera.
        '''
        X = (x*cos(theta)) - (y*sin(theta)) # Using the rotation matrix to find the transformed coordinates,
        Y = (x*sin(theta)) + (y*cos(theta)) # i.e [X][T] = [X'], X'= Transformed coordinates, X- Original Coordinates, T-rotation matrix 
        return X, Y

    def StateEstimator(self): 
        ''' This function updates the estimated state(x, y, z, yaw) of the drone '''
        if self.Camera_Measurement():                                        # if camera catches the drone position
            self.estimated_position_and_yaw = self.measured_position_and_yaw # update estimate state
            return True                                                      # if update is succesfull, return True
        else:
            return False 

    def Adjust_PID(self):
        '''Calculating output for P and D'''
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
        '''Controlling the roll, pitch, yaw and throttle values to feed the drone'''
        if not self.start_pid:
            if 10 <= self.estimated_position_and_yaw[3] and self.estimated_position_and_yaw[3] <= drone_height:
                self.start_pid = True

        if self.wait.Wait():
            if self.StateEstimator():
                self.error                   = self.setpoint - self.estimated_position_and_yaw
                self.error[0], self.error[1] = self.Transform_coordinates(self.error[0], self.error[1], self.Yaw)
                self.error[2]               *= 100
                self.error                   = np.round(self.error)
                    
                self.derr                    = (self.error - self.prev_error)/self.elapsed_time
                self.P_output                = self.Kp * self.error
                self.D_output                = self.Kd * self.derr
                self.I_output                = self.Ki * self.error_sum

                self.Adjust_PID()

                self.prev_error              = self.error
                self.error_sum               = self.error_sum + (self.error*self.elapsed_time)

                self.result                  = self.P_output + self.D_output + self.I_output
                self.result                  = self.result + self.NEUTRAL

                self.Input                   = np.fmin(self.HIGH, np.fmax(self.LOW, self.result))
                self.elapsed_time            = self.delta_time

                self.Send_Input()

            else:
                self.elapsed_time += self.delta_time
                self.Send_Input()

    def Send_Input(self):
        '''Sending the values to control the drone'''
        if self.start_pid:
            self.pluto.setRC({"roll":int(self.Input[0]),"pitch":int(self.Input[1]),"yaw":int(self.Input[2]),"throttle":int(self.Input[3])})
        else:
            self.pluto.setRC({"roll":int(1500),"pitch":int(1500),"yaw":int(1500),"throttle":int(1500)})

    def Set_Multiple_Targets(self, targets):
        '''Setpoints for the drone to move to'''
        if isinstance(targets, list):
            self.target_points = targets
        else:
            print("ERROR: targets should be a list")

    def Change_Target(self):
        '''Changing the target point for the drone to move to'''
        if hasattr(self, "target_points"):
            if len(self.target_points) > 0 and self.check_for_changing_setpoint():
                self.setpoint = np.array(self.target_points[0])
                self.target_points.pop(0)

    def check_for_changing_setpoint(self):
        '''Checking the conditions for changing the setpoint'''
        if max(abs(self.setpoint - self.estimated_position_and_yaw)[:2]) <= self.x_y_error_tolerance and max(abs(self.derr[:2])) <= self.x_y_derr_tolerance: # almost reached the given target point
            self.frame_for_hovering += 1
            if self.frame_for_hovering >= self.minimum_frames_for_hovering: # drone hovered for some limit
                self.frame_for_not_hovering = 0
                self.frame_for_hovering     = 0
                return True                                                 # drone reached the given the setpoint, hence True
            else:
                return False
        else:                                                               # drone away from the setpoint
            self.frame_for_not_hovering += 1
            if self.frame_for_not_hovering >= self.minimum_frames_for_not_hovering:
                self.frame_for_not_hovering = 0
                self.frame_for_hovering     = 0
            return False                                                    # drone did not reach the given the setpoint, hence False


if __name__ == "__main__":

    starting_delay = Delay(2)
    p_drone = MyDrone(camera_ID=0)
    p_drone.Set_Multiple_Targets([[100, 30, 0, drone_height], [-100, 30, 0, drone_height], [-100, -70, 0, drone_height], [100, -70, 0, drone_height], [100, 30, 0, drone_height], [0, 0, 0,drone_height]])
    while True:
        if starting_delay.Wait_Once():
            p_drone.controller()
            p_drone.Change_Target()
            '''Keyboard Control for the drone'''
            key = cv2.waitKey(1) & 0xff
            if key  == ord("q") or key == ord("b"):
                break
            elif key == ord("m"):                  # ARM
                p_drone.pluto.ARM()        
            elif key == ord("n"):                  # DISARM
                p_drone.pluto.DISARM()
            elif key == ord("c"):                  # LAND
                p_drone.pluto.Land()               
            elif key == ord("x"):                  # TAKEOFF
                p_drone.pluto.TakeOff()            
            elif key==ord("y"):                    # ALTITUDE HOLD OFF
                p_drone.pluto.AltitudeHold_OFF() 
            elif key==ord("u"):                    # ALTITUDE HOLD ON
                p_drone.pluto.AltitudeHold_ON() 
            elif key==ord("p"):                    # HEAD FREE OFF
                p_drone.pluto.HeadFree_OFF()    
            elif key==ord("o"):                    # HEAD FREE ON
                p_drone.pluto.HeadFree_ON()    

p_drone.__del__()