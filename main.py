import cv2
import cv2.aruco as aruco
import numpy as np
from dataclasses import dataclass
from configuration import cam_id
from singleton_logger import SingletonLogger
import configuration as conf
from robot_scenarios import PyramidBuilder
from typing import List, Dict, Optional

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
    """Калибровка камеры с использованием шахматной доски"""
    pattern_size = (7, 7)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

    obj_points, img_points = [], []
    ret, corners = cv2.findChessboardCorners(img, (7,6), None)

    if ret:
        corners2 = cv2.cornerSubPix(
            cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
            corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        obj_points.append(objp)
        img_points.append(corners2)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, img.shape[::-1], None, None)

        if ret:
            mean_error = 0
            for i in range(len(obj_points)):
                imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                mean_error += error

            logger.info(f"Средняя ошибка репроекции: {mean_error / len(obj_points)} пикселей")
            np.savez('camera_calibration.npz',
                     camera_matrix=mtx,
                     dist_coeffs=dist,
                     reprojection_error=mean_error / len(obj_points))
            return mtx, dist

    logger.error("Ошибка калибровки!")
    return None, None

def get_marker_by(id_, markers):
    return next((m for m in markers if m.id == id_), None)

def detect_and_process_markers():
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        logger.error(f"Ошибка! Камера id={cam_id} не подключена!")
        exit()

    # Инициализация детекторов
    parameters = aruco.DetectorParameters()
    detectors = {
        '4X4': aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_4X4_250), parameters),
        '5X5': aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_5X5_250), parameters),
        '6X6': aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_6X6_250), parameters)
    }

    # Загрузка или калибровка параметров камеры
    try:
        calib_data = np.load('camera_calibration.npz')
        camera_matrix = calib_data['camera_matrix']
        dist_coeffs = calib_data['dist_coeffs']
        logger.info("Параметры камеры загружены из файла")
    except:
        logger.warning("Файл калибровки не найден, выполняется калибровка...")
        ret, frame = cap.read()
        if ret:
            camera_matrix, dist_coeffs = calibr(aruco.getPredefinedDictionary(aruco.DICT_4X4_250), frame)
        else:
            logger.error("Не удалось получить кадр для калибровки")
            exit()

    # Инициализация сборщика пирамиды
    builder = PyramidBuilder()
    builder.camera_matrix = camera_matrix
    builder.dist_coeffs = dist_coeffs

    logger.info("Нажмите 'q' для выхода или 's' для начала сборки")
    building = False

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.error("Ошибка: Не удалось получить кадр")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        all_markers = []

        # Детекция маркеров всех типов
        for det_name, detector in detectors.items():
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.05, camera_matrix, dist_coeffs)

                for i in range(len(ids)):
                    marker = Marker(
                        id=int(ids[i][0]),
                        position=p3_from_list(tvecs[i][0]),
                        oriental=p3_from_list(rvecs[i][0])
                    )
                    all_markers.append(marker)
                    cv2.drawFrameAxes(
                        frame, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.05
                    )
                aruco.drawDetectedMarkers(frame, corners, ids)

        # Обработка команд
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and not building:
            building = True
            logger.info("Начало сборки пирамиды...")

        # Если идет процесс сборки
        if building and all_markers:
            pyramid_complete = builder.process_markers(all_markers)
            if pyramid_complete:
                logger.info("Пирамида успешно собрана!")
                building = False

        # Отображение информации
        if builder.pyramid_center:
            cv2.putText(frame, f"Уровень: {builder.current_level}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

        cv2.imshow('ArUco Detection', frame)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        detect_and_process_markers()
    except Exception as e:
        logger.error(f"Критическая ошибка: {e}")
        raise