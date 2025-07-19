import numpy as np
import configuration as conf

from typing import Dict, List
import time
from models import Point3, Marker
from robot_client import send_to_arduino_and_get_response
from command import Move, SetRotate, VacuumOn, VacuumOff
from singleton_logger import  SingletonLogger

logger = SingletonLogger()

class PyramidBuilder:
    def __init__(self):
        self.max_zindex = 5
        self.current_level = 0
        self.safe_height = self.max_zindex * conf.marker_z + conf.field_z

        self.corner_ids = [0, 1, 2, 3]
        self.target_ids = [40, 50, 60, 70, 80]
        self.pyramid_center = None

    def _rotation_vector_to_angle(self, rotation: 'Point3') -> float:
        """Конвертирует вектор вращения в угол вокруг оси Z в градусах"""
        # Вычисляем длину вектора вращения
        theta = np.sqrt(rotation.x**2 + rotation.y**2 + rotation.z**2)

        # Нормализуем ось вращения
        if theta < 1e-6:
            return 0.0

        axis = np.array([rotation.x, rotation.y, rotation.z]) / theta

        # Преобразуем в угол вокруг оси Z (проекция)
        z_angle = np.arctan2(axis[1], axis[0]) * 180 / np.pi

        # Приводим к диапазону 0-180
        normalized_angle = z_angle % 180
        return normalized_angle

    def calculate_working_area(self, markers: Dict[int, Marker]):
        """Вычисляет центр рабочей зоны по угловым маркерам"""
        corners = [markers[id].position for id in self.corner_ids if id in markers]

        if len(corners) >= 3:
            center = np.mean([(p.x, p.y) for p in corners], axis=0)
            self.pyramid_center = Point3(center[0], center[1], 0)
            logger.debug(f"Центр рабочей зоны: {self.pyramid_center}")

    def safe_move(self, pos: Point3):
        """Безопасное перемещение через безопасную высоту"""
        safe_pos = Point3(pos.x, pos.y, self.safe_height)
        send_to_arduino_and_get_response(Move(safe_pos))
        send_to_arduino_and_get_response(Move(pos))

    def pick(self, pos: Point3):
        self.safe_move(pos)
        send_to_arduino_and_get_response(VacuumOn())
        time.sleep(0.5)
        self.safe_move(Point3(pos.x, pos.y, self.safe_height))

    def place(self, pos: 'Point3', angle_deg: float = None):
        """Установка маркера в указанную позицию с поворотом
        
            Args:
                pos: Целевая позиция (Point3)
                angle_deg: Угол поворота в градусах [0-180] или None если не нужно вращать
            """
        if angle_deg is not None:
            clamped_angle = max(0, min(180, angle_deg))
            send_to_arduino_and_get_response(SetRotate(int(clamped_angle)))

        self.safe_move(pos)
        send_to_arduino_and_get_response(VacuumOff())
        time.sleep(0.3)
        self.safe_move(Point3(pos.x, pos.y, self.safe_height))

    def process_markers(self, markers: List[Marker]) -> bool:
        """
        Основной метод обработки маркеров.
        Возвращает True если пирамида собрана, иначе False.
        
        Алгоритм:
        1. Сначала убираем все лишние маркеры из центра
        2. Затем собираем пирамиду из целевых маркеров
        """
        if not markers:
            return False

        # Конвертируем в словарь для удобства
        markers_dict = {m.id: m for m in markers}

        # Первичная калибровка (если еще не сделана)
        if self.pyramid_center is None:
            self.calculate_working_area(markers_dict)
            if self.pyramid_center is None:
                logger.warning("Не найдены угловые маркеры для калибровки!")
                return False

        # 1. Фаза очистки: убираем лишние маркеры не из target_ids
        self._remove_extra_markers(markers_dict)

        # 2. Фаза сборки: строим пирамиду из целевых маркеров в порядке убывания ID
        for marker_id in sorted(self.target_ids, reverse=True):  # Сортируем ID по убыванию
            if marker_id in markers_dict:
                marker = markers_dict[marker_id]
                logger.info(f"Сборка: обработка маркера {marker_id} на позиции {marker.position}")

                # Захватываем маркер
                self.pick(marker.position)

                # Вычисляем угол поворота
                angle_deg = self._rotation_vector_to_angle(marker.oriental)

                goida_z = conf.field_z + self.current_level * (conf.marker_z * (1 + 0.1 * (max(self.target_ids) - marker_id)))
                # Целевая позиция для установки
                target_pos = Point3(
                    self.pyramid_center.x,
                    self.pyramid_center.y,
                    goida_z
                )

                # Устанавливаем маркер в пирамиду
                self.place(target_pos, angle_deg)

                # Обновляем состояние
                self.target_ids.remove(marker_id)
                self.current_level += 1
                logger.info(f"Добавлен уровень {self.current_level}")

        return len(self.target_ids) == 0

    def _remove_extra_markers(self, markers_dict: Dict[int, Marker]):
        """Убирает лишние маркеры не из target_ids"""
        # Определяем зону очистки (круг радиусом 100 мм вокруг центра)
        cleanup_radius = 100  # мм

        for marker_id, marker in list(markers_dict.items()):
            # Пропускаем угловые и целевые маркеры
            if marker_id in self.corner_ids or marker_id in self.target_ids:
                continue

            # Проверяем находится ли маркер в зоне очистки
            distance_to_center = np.sqrt(
                (marker.position.x - self.pyramid_center.x)**2 +
                (marker.position.y - self.pyramid_center.y)**2
            )

            if distance_to_center < cleanup_radius:
                logger.info(f"Убираем лишний маркер {marker_id} из центра")

                # Захватываем маркер
                self.pick(marker.position)

                # Переносим в зону сброса (за пределы рабочей области)
                dump_pos = Point3(
                    self.pyramid_center.x + 200,  # 200 мм правее центра
                    self.pyramid_center.y + 200,  # 200 мм выше центра
                    self.safe_height
                )

                # Сбрасываем маркер
                self.place(dump_pos)

                # Удаляем из словаря, чтобы больше не обрабатывать
                del markers_dict[marker_id]