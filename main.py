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

def calibr(aruco_dict, img):
    pattern_size=(7, 7)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    obj_points, img_points = [], []
    ret, corners = cv2.findChessboardCorners(img, (7,6), None)
    if ret:
        obj_points.append(objp)

        # Уточнение позиции углов
        corners2 = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1),
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        img_points.append(corners2)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img.shape[::-1][1], img.shape[::-1][0], None, None)
    mean_error = 0
    for i in range(len(obj_points)):
        imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    print(f"Средняя ошибка репроекции: {mean_error / len(obj_points)} пикселей")
    np.savez('camera_calibration.npz',
             camera_matrix=mtx,
             dist_coeffs=dist,
             reprojection_error=mean_error / len(obj_points))
    return np.savez
def get_marker_by(id_, markers):
    return list(filter(lambda m: m.id == id_, markers))[0]

def detect_img_markers():
    cap = cv2.VideoCapture(cam_id)

    if not cap.isOpened():
        logger.error(f"Ошибка! Камера id={cam_id} не подключена!")
        exit()
    parameters = aruco.DetectorParameters()
    aruco_dict_4X4 = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    detector_4X4 = aruco.ArucoDetector(aruco_dict_4X4, parameters)
    aruco_dict_5X5 = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
    detector_5X5 = aruco.ArucoDetector(aruco_dict_5X5, parameters)
    aruco_dict_6X6 = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
    detector_6X6 = aruco.ArucoDetector(aruco_dict_6X6, parameters)
    # camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
    # dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    logger.debug("Configure ar/uco")

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.error("Ошибка: Не удалось получить кадр")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_4X4, ids_4X4, rejected_4X4 = detector_4X4.detectMarkers(gray)
        corners_5X5, ids_5X5, rejected_5X5 = detector_5X5.detectMarkers(gray)
        corners_6X6, ids_6X6, rejected_6X6 = detector_6X6.detectMarkers(gray)
        if ids_4X4 is not None:
            camera_parameters = calibr(aruco_dict_4X4, frame).load('camera_calibration.npz')
            aruco.drawDetectedMarkers(frame, corners_4X4, ids_4X4)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners_4X4, 0.05, )

            markers = [Marker(id, pos, orient) for id,pos,orient in zip(ids_4X4, rvecs,tvecs)]

            # Отрисовка осей для каждого маркера
            for m in markers:
                cv2.drawFrameAxes(frame, camera_parameters['camera_matrix'], camera_parameters['dist_coeffs'],
                                  m.position, m.oriental, 0.05)
                # Вывод информации о позиции и ориентации
                logger.info(f"Marker found: {m}")
                
            top_left = get_marker_by(2, markers)
            top_right = get_marker_by(3, markers)
            bottom_left = get_marker_by(1, markers)
            bottom_right = get_marker_by(0, markers)
            logger.info(f"Получены углы:\n\tВерхний левый:{top_left}\n\tВерхний правый:{top_right}\n\tНижний левый:{bottom_left}\n\tНижний правый:{bottom_right}")
            
        # for 5X5
        if ids_5X5 is not None:
            aruco.drawDetectedMarkers(frame, corners_5X5, ids_5X5)
            camera_parameters = calibr(aruco_dict_5X5, frame).load('camera_calibration.npz')
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners_5X5, 0.05, camera_parameters['camera_matrix'], camera_parameters['dist_coeffs'])

            markers = [Marker(id, pos, orient) for id, pos, orient in zip(ids_5X5, rvecs, tvecs)]

            # Отрисовка осей для каждого маркера
            for m in markers:
                cv2.drawFrameAxes(frame, camera_parameters['camera_matrix'], camera_parameters['dist_coeffs'],
                                  m.position, m.oriental, 0.05)
        # for 6X6
        if ids_6X6 is not None:
            camera_parameters = calibr(aruco_dict_6X6, frame).load('camera_calibration.npz')
            aruco.drawDetectedMarkers(frame, corners_6X6, ids_6X6)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners_6X6, 0.05,  camera_parameters['camera_matrix'], camera_parameters['dist_coeffs'])

            markers = [Marker(id, pos, orient) for id, pos, orient in zip(ids_6X6, rvecs, tvecs)]

            # Отрисовка осей для каждого маркера
            for m in markers:
                cv2.drawFrameAxes(frame,  camera_parameters['camera_matrix'], camera_parameters['dist_coeffs'],
                                  m.position, m.oriental, 0.05)
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