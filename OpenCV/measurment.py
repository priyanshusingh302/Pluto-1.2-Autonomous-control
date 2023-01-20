
import cv2 as cv
from cv2 import aruco
import numpy as np
from statistics import mean, variance
from math import sqrt
import time

D = []
data = []
count = 0
thresh = 2
initial = time.time()
cam_mat = np.array([[671.76272872, 0, 327.8293498],
 [0, 680.86586794, 242.4724014 ],
 [0, 0, 1]])
dist_coef = np.array([[-0.08806987, -0.28966823, 0.00187701, 0.00791557, 0.98369858]])

marker_size = 5.9 #in cm

dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
parameters =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

cap = cv.VideoCapture(1) # this is the magic!

# cap.set(cv.CAP_PROP_SETTINGS,0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv.CAP_PROP_FPS, 60)
cap.set(cv.CAP_PROP_EXPOSURE, 10)
# cap.set(cv.CAP_PROP_FOCUS, 1)


while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = detector.detectMarkers(frame)
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, marker_size, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            # Draw the pose of marker
            poit = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 3,2)

            cv.putText(
                frame,
                f"id: {ids[0]} Dist : {round(tVec[i][0][2], 2)}",
                (top_right[0], top_right[1]),
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 255),
                2,
                cv.LINE_AA,
            )
            print(tVec[i][0][2])
            cv.putText(
                frame,
                f"x : {round(tVec[i][0][0], 1)}, y : {round(tVec[i][0][1], 1)}",
                (bottom_right[0], bottom_right[1]),
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (200, 0, 0),
                2,
                cv.LINE_AA,
            )
            if count < thresh:
                count += 1
                D.append(tVec[i][0][2])
            else:
                t = time.time() - initial
                initial = time.time()
                count = 0
                data.append({"max":max(D), "min":min(D), "mean":mean(D), "variance":variance(D), "standard deviation":sqrt(variance(D)), "time": 1/t})
            # print(ids, "  ", corners)
    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()
for i in data:
    print("\n", i)

print("\n")

mean_data = [i["mean"] for i in data]
print("max", max(mean_data), "min", min(mean_data), "mean", mean(mean_data), "variance", variance(mean_data), "standard deviation", sqrt(variance(mean_data)), sep="\n")

