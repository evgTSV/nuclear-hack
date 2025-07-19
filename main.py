import cv2
import cv2.aruco as aruco
import numpy as np
import tkinter as tk
from tkinter import ttk, scrolledtext
from PIL import Image, ImageTk
from threading import Thread, Event
from io import StringIO
from singleton_logger import SingletonLogger
from models import Point3, Marker, p3_from_list
from builder import  PyramidBuilder
import configuration as conf
import time

logger = SingletonLogger()

def calibr(aruco_dict, img):
    pattern_size=(7, 6)
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

class ArucoPyramidApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ArUco Pyramid Builder")

        # Инициализация системы
        self.setup_log_capture()
        self.builder = PyramidBuilder()
        self.building_event = Event()
        self.is_running = False
        self.calibrated = False
        self.cap = None

        # Создание интерфейса
        self.create_ui()
        self.setup_log_updater()

    def setup_log_capture(self):
        """Настройка захвата логов для GUI"""
        self.log_stream = StringIO()
        logger.add_custom_stream(self.log_stream)
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
            text="Start Camera",
            command=self.start_processing
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.build_btn = ttk.Button(
            control_frame,
            text="Start Building",
            command=self.start_building,
            state=tk.DISABLED
        )
        self.build_btn.pack(side=tk.LEFT, padx=5)

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

    def load_calibration(self, detectors, frame):
        """Загрузка параметров калибровки"""
        try:
            # calibr(detectors['4X4'], frame)
            self.builder.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
            self.builder.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
            logger.info("Camera calibration loaded successfully")
        except Exception as e:
            logger.warning(f"Failed to load calibration: {str(e)}")
            # Используем параметры по умолчанию
            self.builder.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
            self.builder.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

    def start_processing(self):
        """Запуск обработки видео"""
        if not self.is_running:
            self.cap = cv2.VideoCapture(conf.cam_id)
            if not self.cap.isOpened():
                logger.error("Failed to open camera")
                return

            self.is_running = True
            self.start_btn.config(state=tk.DISABLED)
            self.build_btn.config(state=tk.NORMAL)
            Thread(target=self.video_processing_loop, daemon=True).start()
            logger.info("Video processing started")

    def start_building(self):
        """Запуск процесса сборки пирамиды"""
        if not self.building_event.is_set():
            self.building_event.set()
            self.build_btn.config(text="Stop Building")
            logger.info("Pyramid building started")
        else:
            self.building_event.clear()
            self.build_btn.config(text="Start Building")
            logger.info("Pyramid building stopped")

    def video_processing_loop(self):
        """Основной цикл обработки видео"""
        try:
            parameters = aruco.DetectorParameters()
            detectors = {
                '4X4': aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_4X4_250), parameters),
                '5X5': aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_5X5_250), parameters),
                '6X6': aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_6X6_250), parameters)
            }

            while self.is_running:
                ret, frame = self.cap.read()
                if not ret:
                    logger.warning("Failed to capture frame")
                    continue
                    
                if not self.calibrated:
                    self.load_calibration(detectors, frame)
                    self.calibrated = True

                # Обработка кадра
                processed_frame, markers = self.process_frame(frame, detectors)

                # Если активирована сборка и есть маркеры
                if self.building_event.is_set() and markers:
                    pyramid_complete = self.builder.process_markers(markers)
                    if pyramid_complete:
                        logger.info("Pyramid building completed!")
                        self.building_event.clear()
                        self.build_btn.config(text="Start Building")

                self.update_video_display(processed_frame)

        except Exception as e:
            logger.error(f"Video processing error: {str(e)}")
        finally:
            self.stop_processing()

    def process_frame(self, frame, detectors):
        """Обработка одного кадра и детекция маркеров"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        all_markers = []

        # Детекция всех типов маркеров
        for det_name, detector in detectors.items():
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, 0.05, self.builder.camera_matrix, self.builder.dist_coeffs)

                for i in range(len(ids)):
                    marker = Marker(
                        id=int(ids[i][0]),
                        position=p3_from_list(tvecs[i][0]),
                        oriental=p3_from_list(rvecs[i][0])
                    )
                    all_markers.append(marker)

                    # Отрисовка осей маркеров
                    cv2.drawFrameAxes(
                        frame, self.builder.camera_matrix, self.builder.dist_coeffs,
                        rvecs[i], tvecs[i], 0.05
                    )

                # Отрисовка маркеров
                aruco.drawDetectedMarkers(frame, corners, ids)

        # Отображение уровня пирамиды
        if self.builder.pyramid_center:
            cv2.putText(frame, f"Level: {self.builder.current_level}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        return frame, all_markers

    def update_video_display(self, frame):
        """Обновление отображения видео"""
        try:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)

            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
        except Exception as e:
            logger.error(f"Display update error: {str(e)}")

    def stop_processing(self):
        """Остановка обработки видео"""
        if self.is_running:
            self.is_running = False
            self.building_event.clear()
            if self.cap:
                self.cap.release()
            self.start_btn.config(state=tk.NORMAL)
            self.build_btn.config(state=tk.DISABLED, text="Start Building")
            logger.info("Video processing stopped")

    def stop_and_exit(self):
        """Корректное завершение работы"""
        self.stop_processing()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ArucoPyramidApp(root)
    root.protocol("WM_DELETE_WINDOW", app.stop_and_exit)
    root.mainloop()