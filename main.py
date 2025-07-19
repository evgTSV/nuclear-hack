'''import cv2
import cv2.aruco as aruco
import numpy as np
import tkinter as tk
from tkinter import ttk, scrolledtext
from PIL import Image, ImageTk
from threading import Thread
import queue
from builder import PyramidBuilder
from configuration import cam_id
from singleton_logger import SingletonLogger
from models import Point3, Marker, p3_from_list
import configuration as conf

from typing import Dict, List
import time
from models import Point3, Marker
from robot_client import send_to_arduino_and_get_response
from command import Move, SetRotate, VacuumOn, VacuumOff


logger = SingletonLogger()

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
'''
import cv2
import cv2.aruco as aruco
import numpy as np
import tkinter as tk
from tkinter import ttk, scrolledtext
from PIL import Image, ImageTk
from threading import Thread
from io import StringIO
from singleton_logger import SingletonLogger
from builder import PyramidBuilder


builder = PyramidBuilder()

class ArucoPyramidApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ArUco Pyramid Builder")

        # Инициализация логгера
        self.logger = SingletonLogger()
        self.setup_log_capture()

        # Параметры камеры и детекторов
        self.cap = None
        self.is_running = False
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dicts = {
            '4X4': aruco.getPredefinedDictionary(aruco.DICT_4X4_250),
            '5X5': aruco.getPredefinedDictionary(aruco.DICT_5X5_250),
            '6X6': aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        }
        self.parameters = aruco.DetectorParameters()

        # Создание интерфейса
        self.create_ui()
        self.load_calibration()
        self.setup_log_updater()

    def setup_log_capture(self):
        """Настройка захвата логов для GUI"""
        self.log_stream = StringIO()
        self.logger.add_custom_stream(self.log_stream)
        self.log_lines = []

    def create_ui(self):
        """Создание элементов интерфейса"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Панель управления
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=5)

        self.start_btn = ttk.Button(
            control_frame,
            text="Start",
            command=self.start_processing
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)

        ttk.Button(
            control_frame,
            text="Exit",
            command=self.stop_and_exit
        ).pack(side=tk.LEFT, padx=5)

        # Основная область
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True)

        # Видео панель
        self.video_frame = ttk.LabelFrame(content_frame, text="Camera View")
        self.video_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.video_label = ttk.Label(self.video_frame)
        self.video_label.pack(fill=tk.BOTH, expand=True)

        # Панель логов
        log_frame = ttk.LabelFrame(content_frame, text="Logs", width=100)
        log_frame.pack(side=tk.RIGHT, fill=tk.BOTH)

        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            state='disabled',
            wrap=tk.WORD
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def setup_log_updater(self):
        """Настройка периодического обновления логов"""

        def update_logs():
            new_logs = self.log_stream.getvalue().splitlines()
            new_lines = [line for line in new_logs if line not in self.log_lines]

            if new_lines:
                self.log_text.config(state='normal')
                for line in new_lines:
                    self.log_text.insert(tk.END, line + "\n")
                    self.log_lines.append(line)
                self.log_text.config(state='disabled')
                self.log_text.see(tk.END)

            self.root.after(100, update_logs)

        update_logs()

    def load_calibration(self):
        """Загрузка параметров калибровки"""
        try:
            #  calib_data = np.load('camera_calibration.npz')
            self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
            self.logger.info("Camera calibration loaded successfully")
        except Exception as e:
            self.logger.warning(f"Failed to load calibration: {str(e)}")

    def start_processing(self):
        """Запуск обработки видео"""
        if not self.is_running:
            self.cap = cv2.VideoCapture(1)
            if not self.cap.isOpened():
                self.logger.error("Failed to open camera")
                return

            self.is_running = True
            self.start_btn.config(state=tk.DISABLED)
            Thread(target=self.video_processing_loop, daemon=True).start()
            self.logger.info("Video processing started")

    def video_processing_loop(self):
        """Основной цикл обработки видео"""
        try:
            while self.is_running:
                ret, frame = self.cap.read()
                if not ret:
                    self.logger.warning("Failed to capture frame")
                    continue

                # Обработка кадра
                processed_frame = self.process_frame(frame)
                self.update_video_display(processed_frame)

        except Exception as e:
            self.logger.error(f"Video processing error: {str(e)}")
        finally:
            self.stop_processing()

    def process_frame(self, frame):
        """Обработка одного кадра"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Детекция всех типов маркеров
        for dict_name, aruco_dict in self.aruco_dicts.items():
            detector = aruco.ArucoDetector(aruco_dict, self.parameters)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None and self.camera_matrix is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.05, self.camera_matrix, self.dist_coeffs)

                for i in range(len(ids)):
                    cv2.drawFrameAxes(
                        frame, self.camera_matrix, self.dist_coeffs,
                        rvecs[i], tvecs[i], 0.05
                    )
                aruco.drawDetectedMarkers(frame, corners, ids)

        return frame

    def update_video_display(self, frame):
        """Обновление отображения видео"""
        try:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)

            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
        except Exception as e:
            self.logger.error(f"Display update error: {str(e)}")


    def stop_processing(self):
        """Остановка обработки видео"""
        if self.is_running:
            self.is_running = False
            if self.cap:
                self.cap.release()
            self.start_btn.config(state=tk.NORMAL)
            self.logger.info("Video processing stopped")

    def stop_and_exit(self):
        """Корректное завершение работы"""
        self.stop_processing()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = ArucoPyramidApp(root)
    root.protocol("WM_DELETE_WINDOW", app.stop_and_exit)
    root.mainloop()