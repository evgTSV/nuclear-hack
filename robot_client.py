import threading

from command import RbCmdKind
from singleton_logger import SingletonLogger
from models import Point3
from command import RbCmd, command_to_str

import serial
import serial.tools.list_ports
import time
import configuration as conf

logger = SingletonLogger()

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            ser = serial.Serial(port.device, 115200, timeout=1)
            logger.info(port.device)
            time.sleep(2)
            ser.flushInput()
            ser.write(b'PING\n')
            print(1)
            response = ser.readline().decode().strip()
            print(response)
            if response == 'PONG':
                logger.info(f"Arduino обнаружена на {port.device}")
                return ser
            ser.close()
        except serial.SerialException:
            continue
    logger.error("Arduino не найдена")
    return None

serial_lock = threading.Lock()

def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def rb2abs_coord(pos: Point3):
    Point3(
        pos.x + conf.offsets[0],
        pos.y + conf.offsets[1],
        pos.z + conf.offsets[2]
    )

def send_to_arduino_and_get_response(command: RbCmd):
    ser = find_arduino_port()
    if ser is None:
        logger.warning("Ожидание подключения ардуино")
        return "null"
    logger.info(f"Принята команда: {command}")

    command_str = command_to_str(command)

    with serial_lock:
        ser.write((command_str + '\n').encode())
        while True:
            response = ser.readline().decode().strip()
            if response:
                logger.info(f"Ответ от Arduino: {response}")
                return response
            else:
                time.sleep(0.1)