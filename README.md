
 # Inter IIT 11.0 - Drone Control and Swarm Communication

[![Youtube-Video](https://img.youtube.com/vi/PdnE_47Gra4/0.jpg)](https://www.youtube.com/watch?v=PdnE_47Gra4)

## Project Overview

This repository contains the code and documentation for the Inter IIT Tech Meet 11.0 project, focusing on drone control and swarm communication.

### Abstract

The problem statement designed by Drona Aviation Pvt Ltd aims to ease the usage of the Pluto drone by making a python wrapper for manual control of the drone, taking ideas from the existing ROS wrappers and the resources for understanding the communication between the drone and the information sources which can differ.

The problem statement also aims to design an algorithm for autonomous waypoint traversal of the drone in an indoor environment using Computer vision using the miraculous Aruco markers using pose estimation, distance estimation, etc. This also introduces the need for control systems using different controllers and filters.

The final sub-problem also introduces a list of a swarm of drones but uses just two drones in synchronous such that once the first drone reaches a particular setpoint the other drone tries to go to the previous setpoint of the first drone.

Overall, the problem statement makes a complete introduction to understanding and implementing the following things:
- Communication methods of Pluto drone and MSP packets for data transmission
- Using Object-oriented programming and defining classes for data abstraction and particular methods required in varying operations.
- Keyboard control to define motions for throttle, roll, pitch, yaw, etc.
- RGB camera usage, camera calibration, pose estimation using Aruco markers, distance predictions, projective transformations, dilation corrections, and a lot more related to optics and photogrammetry.
- Control systems and usage of different controllers like PID, LQR, etc for autonomous waypoint missions for drones.
- Communication of data serially/wirelessly between the microcontroller and the secondary drone for data transmission.
- Synchronizing the motion of two drones together in the same environment.

### Implementation

#### Task 1 - Python Wrapper for Manual Control

The main aim of the first task is to develop a python wrapper from the existing ROS packages and the existing communication methods of the Pluto drone with external peripherals namely the MSP packages. The drone has six degrees of freedom clearly namely the three linear coordinates and the three orientations along each plane or more specifically the Euler angles in the 3D space named pitch, roll, and yaw in the world of drones.

The Python wrapper's final aim is to allow for manual control of the drone for different movements like the roll, pitch, yaw, and throttle of the drone to control the attitude. Simply this means the drone can be controlled using a keyboard or from commands using the serial port using appropriate interfaces and codes.

The python wrapper implements a communication pipeline between a laptop and Pluto drone using the Telnet protocol. The pipeline starts with a connection to the Pluto drone with a specified host and port through the connect function, which opens a Telnet connection and starts a new thread monitorThread to listen to incoming data from the drone. The data is received by the monitorSerialPort function and parsed into individual messages consisting of a header, message size, command, data payload, and checksum. The parsed messages are stored in the responses dictionary. The pipeline can be terminated by calling the disconnect function, which stops the monitorThread and closes the Telnet connection.

- **sendData:** sends a packet of data over Telnet
- **waitForResponse:** waits until the requested data is received and then returns the data
- **encodePacket:** creates an MSP packet and sends it over the Telnet connection
- **getData:** encapsulates the process of sending a command to get data and returning the data when it is received
- **setRC:** sends RC values to the drone to control its flight
- **setCommand:** sends different commands to the drone to perform specific actions such as takeoff, landing, or flipping
- **setThrottle(value):** Sets the throttle value of the quadcopter in PWM units with a range of 900 to 2100.
- **setPitch(value):** Sets the pitch value of the quadcopter in PWM units with a range of 900 to 2100.
- **setRoll(value):** Sets the roll value of the quadcopter in PWM units with a range of 900 to 2100.
- **setYaw(value):** Sets the yaw value of the quadcopter in PWM units with a range of 900 to 2100.
- **ARM():** Arms the quadcopter by setting a value of 1500 to the "aux4" channel.
- **DISARM():** Disarms the quadcopter by setting a value of 1000 to the "aux4" channel.
- **AltitudeHold_ON():** Turns on the altitude hold mode by setting a value of 1500 to the "aux3" channel.
- **AltitudeHold_OFF():** Turns off the altitude hold mode by setting a value of 1000 to the "aux3" channel.
- **DevMode_ON():** Turns on the developer mode by setting a value of 1500 to the "aux2" channel.
- **DevMode_OFF():** Turns off the developer mode by setting a value of 1000 to the "aux2" channel.
- **HeadFree_ON():** Turns on the head-free mode by setting a value of 1500 to the "aux1" channel.
- **HeadFree_OFF():** Turns off the head-free mode by setting a value of 1000 to the "aux1" channel.
- **TakeOff():** Commands the quadcopter to take off by calling the setCommand(1) method.
- **Land():** Commands the quadcopter to land by calling the setCommand(2) method.
- **BackFlip():** Commands the quadcopter to perform a backflip by calling the setCommand(3) method.
- **FrontFlip():** Commands the quadcopter to perform a frontflip by calling the setCommand(4) method.
- **RightFlip():** Commands the quadcopter to perform a rightflip by calling the setCommand(5) method.
- **LeftFlip():** Commands the quadcopter to perform a leftflip by calling the setCommand(6) method.
- **getAltitude():** Returns the estimated altitude of the quadcopter in centimeters.
- **getVariometer():** Returns the estimated vertical velocity of the quadcopter in cm/s.
- **getAcc():** Returns the raw data from the accelerometer of the quadcopter in the format [X-axis, Y-axis, Z-axis].
- **getGyro():** Returns the raw data from the gyroscope of the quadcopter in the format [X-axis, Y-axis, Z-axis].
- **getMag():** Returns the raw data from the magnetometer of the quadcopter in the format [X-axis, Y-axis, Z-axis].
- **getRoll():** Returns the roll of the quadcopter in degrees.
- **getPitch():** Returns the pitch of the quadcopter in degrees.
- **getYaw():** Returns the yaw of the quadcopter in degrees.

#### Task 2 - Autonomous Waypoint Traversal using Computer Vision

For the identification and tracking of drones in mid-air, a single monocular camera from the top view has been used. The specs of the camera used by us are as follows:
- FPS: 60
- Resolution: 1920x1080
It is responsible for tracking the position of the drone. The position of the drone is estimated by the Aruco marker that is stuck on the top of the drone. Aruco marker used by us has the following params:
- Aruco marker size: 5.9cm.
- Aruco marker ID: 0
The camera is calibrated to estimate the x,y, and depth values of the aruco marker. Detection of the aruco is done using the Python OpenCV library. The coordinates of the aruco obtained by the camera are then transformed to the coordinates with respect to the drone as to command the drone, it needs the points in its frame.

- First all the parameters of drone and camera are initialized and initial setpoint is set
- The `camera_measurement` function runs in a loop, getting the image frame and updating the measured position of the drone.
- These measured position values then pass through the `state_estimation` block which gives the estimated states
- The estimated position and the desired position are then used to feed the PID algorithm to provide roll, pitch, yaw and throttle values, this values are later send to the drone using pluto wrapper
- Once the drone reaches the desired setpoint, and holds there for some time, a new set point will be set for the target setpoint based on the remaining setpoints needed to reach in the provided setpoints list.
- One all setpoints reached, the drone will land
In the `Image Capturing.py` file, we click the images of the 7X7 chessboard. Then checking if the “images” directory exists or not, if it does not exist then create it. Then creating a user-defined function `detect_checker_board()` to detect the chessboard in an image. Inside this function, we use `cv.findChessboardCorners()` and pass a grayscale image of the chessboard and its dimension. If the board is detected in the image then we draw the corners of the chessboard using `cv.drawChessboardCorners()` and return this image. Each frame that we get from the camera is converted into grayscale images and we pass these images into the `detect_checker_board()` function, this function returns the image with corners drawn on it so we additionally add text to it using `cv.putText()` as “saved_img”. Images are saved by clicking the ‘s’ key on the keyboard.

In the `Cam Calibration.py` file, we first define the chessboard dimension and square size in millimeters, then we check the directory's path where the calibrated data will be saved, if the directory is not existing then we create one. Then we use the images that were saved from the `Image Capturing.py` script to again detect and draw the corners of the chessboard. Then we calibrate the camera using `cv.calibrateCamera()` function and pass the 3D object points, 2D object points, and grayscale image. This function returns the camera matrix, distance coefficient, translation vector, and rotation vector. And finally, then we load all this data into the `calib_data.npz` file.

In the `Distance Measurement.py` file, we use this `calib_data.npz` file, load its data, define the aruco marker size in centimeters, then detected the marker from the frames that we are capturing from the camera, if the marker is detected then we estimate its position using `cv.estimatePoseSingleMarkers()` and pass marker corners, marker size, camera matrix, and distance coefficient and then we draw the boundary of the marker and its axes using `cv.polyLines()` and `cv.drawFrameAxes()`. Then using the translation vector we get the distance that is the z coordinate of the marker from the plane of the camera and then we add text i.e., ID number and Distance to each frame.

#### Task 3 - Communication with 2nd Drone

The ESP32 code is written in C++ using the Arduino IDE. It sets up a connection to a Wi-Fi network with the SSID and password. The code uses the built-in LED on the ESP32 board and blinks it until the device successfully connects to the Wi-Fi network.

Once connected, the code sets up a client connection to a server at some IP address and port. The loop function reads incoming data from both the serial and client connections and writes the received data to the other connection. It flushes the client connection and stops it after the data has been sent.

- First, the camera object is initialized with the required parameters of detection and camera specification. Both drone objects with their respective communication protocol are defined along with the sampling time of their PID and their initial setpoint. Then a list of Rectangle corner coordinates is supplied to the first PlutoPluto drone.
- Then the `camera` object member function is run in a loop that takes the camera feed, detects the provided aruco markers, estimates their position, shows the camera image feed, and updates the detected aruco position as a dictionary with their corresponding aruco id as a key. This function also returns a “success” dictionary which contains boolean values for the provided aruco ids as keys of whether they are detected in that frame or not.
- These values of aruco id position are then fed to their corresponding drone object controller function. this controller function will make the drone reach the setpoint
- First, the first drone will take off and reach the lower right corner(depending on the order of the rectangle corner list provided to it) of the rectangle and stay there for some time. After that, the follower drone will fly and will acquire the (0, 0) position of the rectangle.
- after the follower drone holds its position for some time, the first drone will change its setpoint and goes to the lower left corner of the rectangle
- After the first drone holds its position for some time, the follower drone will now move to the previous setpoint of the first drone i.e. at the lower right corner of the rectangle.
- After the follower drone holds its position for some time, the first drone will change its setpoint to the upper left corner of the rectangle and move there.
- this alternate cycle will keep on going until the first drone reach its final setpoint(as provided list, in our case we set the last setpoint (0,0) of rectangle), and the follower drone moves to the previous setpoint of the first drone(i.e. previous point of (0,0) of rectangle)
- After that, both drones will finally land.

The main theme of this task is that a set of points will be provided only to the first drone, and the follower drone will chase the previous setpoint of the current setpoint of the first drone. this movement will take alternatively.

### Final Result and Conclusion

#### Task 1

Our idea and application achieved drone take-off quickly with the Python wrapper we created. Only common problems with implementation were faced which were cleared through some trial and error. By then, we successfully connected the drone with our wrapper and sent MSP packets.

#### Task 2

After many failed attempts, we finally got the drone to do what it was meant to do. The drone followed the coordinates which were fed, smoothly while maintaining a fixed altitude. This was achieved by using a single camera. Position control was fine-tuned with different PID gains. Now we could control Pluto’s height and position.

#### Task 3

The second drone successfully communicated with the wrapper through an external WiFi device(ESP32 board with a ping of approx. 100ms) and was made to follow the first drone. But the drone provided to us was defective and was unable to properly takeoff above some altitude. This restricted us to proceed further and test our code. The hardware issue couldn’t be fixed in a limited time. Hence the script was tested even with the faulty drone provided by the company. The script was working nicely and should work with a functional drone

