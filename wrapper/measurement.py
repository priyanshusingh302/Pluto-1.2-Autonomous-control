#!python3
import cv2
import numpy as np
import time



class MyDrone():

    def __init__(self, camera_ID=0):

        #  [roll, pitch, yaw, throttle]
        self.Kp = np.array([0, 0, 0, 0])
        self.Kd = np.array([0, 0, 0, 0])
        self.Ki = np.array([0, 0, 0, 0])
        self.error = np.array([0, 0, 0, 0])
        self.prev_error = np.array([0, 0, 0, 0])
        self.error_sum = np.array([0, 0, 0, 0])
        self.setpoint = np.array([0, 0, 0, 0])
        self.Input = np.array([0, 0, 0, 0])
        self.measured_position_and_yaw = np.array([0, 0, 0, 0])
        self.estimated_position_and_yaw = np.array([0, 0, 0, 0])
        self.NEUTRAL = 1500
        self.HIGH = 2000
        self.LOW = 1000
        self.delta_time = 10e-03  # 10 ms
        self.initial_time = 0
        self.now = 0

        self.camera_matrix = np.array([[2.36854310e+03, 0.00000000e+00, 8.83847869e+02],
                                       [0.00000000e+00, 2.36416743e+03, 6.26598972e+02],
                                       [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.distortion_coefficients = np.array(
            [[-0.53835254, 0.74990547, -0.00897318, 0.00293929, -0.59153803]])
        self.marker_size = 5.9
        self.marker_type = cv2.aruco.DICT_4X4_50
        self.resolution = (1080, 1920)

       

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.capture = cv2.VideoCapture(camera_ID) # this is the magic!
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[0])
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[1])

        # self.pluto = Pluto()
        # self.pluto.connect()

    def __del__(self):
        # self.pluto.disconnect()
        self.capture.release()
        cv2.destroyAllWindows()

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
                        cv2.putText(frame,f"x: {round(TransionalVector[index][0][0])} y: {round(TransionalVector[index][0][1])} z: {round(TransionalVector[index][0][2])}", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 0, 0), 2)
                        # print(TransionalVector[index][0][0],TransionalVector[index][0][1])
                        self.measured_position_and_yaw = np.array(
                            [TransionalVector[index][0][0], TransionalVector[index][0][1], TransionalVector[index][0][2], self.CalculateYaw(TransionalVector)])
                elif isinstance(iD, list):
                    pass
                else:
                    print("Wrong value for iD")
                    exit(0)
            cv2.imshow("image", frame)
        else:
            print("error in openning camera")
            exit(0)

    def StateEstimator(self):

        self.estimated_position_and_yaw = self.measured_position_and_yaw

    def controller(self):

        self.now = time.time()
        if self.now() - self.initial_time >= self.delta_time:

            self.error = self.setpoint - self.estimated_position_and_yaw
            self.Input = self.Kp*self.error + self.Kd * \
                (self.error - self.prev_error)/self.delta_time + \
                self.ki*self.error_sum*self.delta_time
            self.prev_error = self.error
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

        # self.pluto.setRC({"roll":self.Input[0],"pitch":self.Input[1],"yaw":self.Input[2],"throttle":self.Input[3]})
        pass


if __name__ == "__main__":

    obj = MyDrone(camera_ID=2)

    while True:

        obj.Camera_Measurement()
        if cv2.waitKey(1) & 0xff == ord("q"):
            break
