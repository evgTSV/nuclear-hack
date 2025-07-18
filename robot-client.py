import threading
import serial
import serial.tools.list_ports
import time

busy = False
current_client = None
last_command_time = 0
INACTIVITY_TIMEOUT = 30  # 30 секунд

def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            ser = serial.Serial(port.device, 115200, timeout=1)
            print(port.device)
            time.sleep(2)
            ser.flushInput()
            ser.write(b'PING\n')
            print(1)
            response = ser.readline().decode().strip()
            print(response)
            if response == 'PONG':
                print(f"Arduino обнаружена на {port.device}")
                return ser
            ser.close()
        except serial.SerialException:
            continue
    print("Arduino не найдена")
    return None

ser = find_arduino_port()
if ser is None:
    exit()

serial_lock = threading.Lock()


def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def transform_command(command):
    parts = command.strip().split()

    if len(parts) == 4 and (parts[0] in ("MOVE_TO", "MOVE_TO_ROBOT")):
        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
        except ValueError:
            return "Invalid coordinate values"

        if parts[0] == "MOVE_TO_ROBOT":

            x_new = x
            y_new = y
            z_new = z
        else:
            # MOVE_TO: меняем X и Y местами + смещения
            x_new = -y + 552
            y_new = -x + 268
            z_new = z-3
        # Собираем итоговую строку
        return f"{parts[0]} {int(x_new)} {int(y_new)} {int(z_new)}"
    if parts[0] == "TOOL_ROTATE_TO":
        try:
            angle = float(parts[1])
            v = arduino_map(angle, 90, 0, 45, 142)
            return f"{parts[0]} {int(angle)}"

        except ValueError:
            return "Invalid angle values"

    # Если это не MOVE_TO* - возвращаем исходную команду как есть.
    return command

def send_to_arduino_and_get_response(command):
    print(f"[SERVER] Принята команда: {command}")

    if command.startswith("MOVE_TO"):
        command = transform_command(command)

    with serial_lock:
        ser.write((command + '\n').encode())
        while True:
            response = ser.readline().decode().strip()
            if response:
                print(f"[SERVER] Ответ от Arduino: {response}")
                return response
            else:
                time.sleep(0.1)