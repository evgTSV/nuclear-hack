import cv2
import cv2.aruco as aruco
import numpy as np
from dataclasses import dataclass
from configuration import cam_id
from singleton_logger import SingletonLogger
import configuration as conf
from pyramid_builder import PyramidBuilder, Marker3D
from typing import List

logger = SingletonLogger()

@dataclass
class Point3:
    x: float
    y: float
    z: float

def detect_markers(frame, detector, camera_matrix, dist_coeffs) -> List[Marker3D]:
    """Обнаруживает маркеры и возвращает список Marker3D"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return []

    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, conf.marker_size, camera_matrix, dist_coeffs
    )

    markers = []
    for i in range(len(ids)):
        pos = tvecs[i][0] * 1000  # Конвертируем в мм
        rot = rvecs[i][0]
        markers.append(Marker3D(
            id=int(ids[i][0]),
            position=Point3(pos[0], pos[1], pos[2]),
            rotation=Point3(rot[0], rot[1], rot[2])
        ))

    return markers

def visualize(frame, markers: List[Marker3D], builder: PyramidBuilder):
    """Визуализация процесса сборки"""
    for marker in markers:
        # Отрисовка осей маркеров
        cv2.drawFrameAxes(
            frame,
            builder.camera_matrix,
            builder.dist_coeffs,
            np.array([[marker.rotation.x, marker.rotation.y, marker.rotation.z]]),
            np.array([[marker.position.x/1000, marker.position.y/1000, marker.position.z/1000]]),
            0.05
        )

        # Подпись маркера
        cv2.putText(frame, f"ID: {marker.id}",
                    (int(marker.position.x), int(marker.position.y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # Отображение состояния сборки
    if builder.pyramid_center:
        cv2.putText(frame, f"Уровень: {builder.current_level}",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

    cv2.imshow('ArUco Detection', frame)

def load_calibration():
    """Загружает параметры калибровки камеры"""
    try:
        calib_data = np.load('camera_calibration.npz')
        return calib_data['camera_matrix'], calib_data['dist_coeffs']
    except:
        logger.error("Не удалось загрузить параметры калибровки!")
        return None, None

def main():
    # Инициализация детектора ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Загрузка параметров камеры
    camera_matrix, dist_coeffs = load_calibration()
    if camera_matrix is None:
        logger.error("Необходима калибровка камеры!")
        return

    # Инициализация сборщика пирамиды
    builder = PyramidBuilder()
    builder.camera_matrix = camera_matrix
    builder.dist_coeffs = dist_coeffs

    cap = cv2.VideoCapture(cam_id)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                logger.error("Не удалось получить кадр")
                break

            # Детекция маркеров
            markers = detect_markers(frame, detector, camera_matrix, dist_coeffs)

            # Обработка маркеров и сборка пирамиды
            pyramid_complete = builder.process_markers(markers)
            if pyramid_complete:
                logger.info("Пирамида успешно собрана!")
                break

            # Визуализация
            visualize(frame, markers, builder)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.error(f"Ошибка: {str(e)}")
        raise