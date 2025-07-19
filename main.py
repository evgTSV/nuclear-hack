import cv2
import cv2.aruco as aruco
import numpy as np
from dataclasses import dataclass

from configuration import cam_id
from singleton_logger import SingletonLogger
import configuration as conf

logger = SingletonLogger()

@dataclass()
class Point3:
    x: float
    y: float
    z: float
    
def p3_from_list(l):
    return Point3(l[0], l[1], l[2])

@dataclass()
class Marker:
    id: int
    position: Point3
    oriental: Point3
    
def get_marker_by(id_, markers):
    return list(filter(lambda m: m.id == id_, markers))[0]

def detect_img_markers():
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        logger.error(f"Ошибка! Камера id={cam_id} не подключена!")
        exit()
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)

    logger.debug("Configure ar/uco")

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.error("Ошибка: Не удалось получить кадр")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners=corners,
                markerLength=0.05,  # Размер маркера в метрах
                cameraMatrix=camera_matrix,
                distCoeffs=dist_coeffs
            )

            markers = [Marker(id_, p3_from_list(pos), p3_from_list(orient)) for id_,pos,orient in zip(ids,rvecs,tvecs)]

            # Отрисовка осей для каждого маркера
            for m in markers:
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                  m.position, m.oriental, 0.05)
                # Вывод информации о позиции и ориентации
                logger.info(f"Marker found: {m}")
                
            top_left = get_marker_by(2, markers)
            top_right = get_marker_by(3, markers)
            bottom_left = get_marker_by(1, markers)
            bottom_right = get_marker_by(0, markers)
            
            logger.info(f"Получены углы:\n\tВерхний левый:{top_left}\n\tВерхний правый:{top_right}\n\tНижний левый:{bottom_left}\n\tНижний правый:{bottom_right}")

            # Отображение результата
        cv2.imshow('ArUco Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # Освобождение ресурсов
    cap.release()
    cv2.destroyAllWindows()

try:
    detect_img_markers()
except Exception as e:
    logger.error(f"{e}")