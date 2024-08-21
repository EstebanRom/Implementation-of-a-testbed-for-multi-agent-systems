import cv2
from cv2 import aruco as ArUco
import numpy as np

marker_size = 0.11  # 5 cm

# Cargar parámetros de calibración de la cámara
camera_matrix = np.array([[1000.0, 0.0, 320.0], 
                          [0.0, 1000.0, 240.0], 
                          [0.0, 0.0, 1.0]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1), dtype=np.float32)

Cam = cv2.VideoCapture(4)

if not Cam.isOpened():
    print("Error: No se pudo acceder a la cámara.")
else:
    Cam.set(cv2.CAP_PROP_BRIGHTNESS, 0.5) # Brillo
    ArUcoDict = ArUco.getPredefinedDictionary(ArUco.DICT_4X4_250)
    ArUcoParam = ArUco.DetectorParameters()
    ArUcoDetect = ArUco.ArucoDetector(ArUcoDict, ArUcoParam)

    Run = True

    while Run:
        ret, frame = Cam.read()

        if ret:
            ImgBN = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = ArUcoDetect.detectMarkers(ImgBN)

            if len(corners) > 0:
                for i in range(len(corners)):
                    marker_corners = corners[i][0]

                    obj_points = np.array([
                        [0, 0, 0],
                        [marker_size, 0, 0],
                        [marker_size, marker_size, 0],
                        [0, marker_size, 0]
                    ], dtype=np.float32)

                    success, rvec, tvec = cv2.solvePnP(obj_points, marker_corners, camera_matrix, dist_coeffs)

                    if success:
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                        distance = np.linalg.norm(tvec)
                        ptA = tuple(marker_corners[0].astype(int))
                        cv2.putText(frame, f"Distancia = {distance:.2f} m", (ptA[0], ptA[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        coords = tvec.flatten()
                        cv2.putText(frame, f"Coords: x={coords[0]:.2f}, y={coords[1]:.2f}, z={coords[2]:.2f}",
                                    (ptA[0], ptA[1] + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            cv2.imshow('ArUco Detection', frame)
            Key = cv2.waitKey(1)
            if Key == ord('Q'):
                Run = False

    Cam.release()
    cv2.destroyAllWindows()