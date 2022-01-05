import cv2
import numpy as np
import cv2.aruco as aruco
import time

# 注意，需要将opencv的依赖库更新
dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)


# print(dictionary)

# markerImage = np.zeros((200, 200), dtype=np.uint8)
# for i in range(30):
#     markerImage = cv2.aruco.drawMarker(dictionary, i, 200, markerImage, 1)
#     filename = "./armark/" + str(i) + ".png"
#     cv2.imwrite(filename, markerImage)


def find_aruco(img, dct):
    image = cv2.resize(img, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(dct)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    aruco.drawDetectedMarkers(img, corners, ids)


class Camera:
    def __init__(self):
        self.matrix = np.matrix([[900.55307134, 0, 329.52236873], [0, 896.22861821, 266.63626082], [0, 0, 1]])  # 内参数矩阵
        self.dist = np.matrix(
            [[-4.91349889e-01, 6.83251536e-01, 1.82902783e-03, 1.28806303e-03, -2.03410762e+00]])

    def calibration(self, image):
        u, v = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.matrix, self.dist, (u, v), 0, (u, v))
        map_x, map_y = cv2.initUndistortRectifyMap(self.matrix, self.dist, None, new_camera_matrix, (v, u), 5)
        dst2 = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR)
        return dst2


if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    font = cv2.FONT_HERSHEY_SIMPLEX
    c = Camera()

    while True:
        ret, frame = cap.read()
        h1, w1 = frame.shape[:2]
        image = c.calibration(frame)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, c.matrix, c.dist)

            (rvec-tvec).any()

            for i in range(rvec.shape[0]):
                aruco.drawAxis(image, c.matrix, c.dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                aruco.drawDetectedMarkers(image, corners)
            cv2.putText(image, "Id: " + str(ids), (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            cv2.putText(image, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow("image", image)
        key = cv2.waitKey(1)
        if key == 27:
            print("esc breaking")
            cv2.destroyAllWindows()
            break
        if key == ord(' '):
            filename = "./detected/" + str(time.time())[:10] + '.img'
            cv2.imwrite(filename, image)
