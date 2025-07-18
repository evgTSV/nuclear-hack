import cv2
import robot_client as rb

cap = cv2.VideoCapture(1) # пробуем встроенную камеру
if not cap.isOpened():
    print("Ошибка! Камера не подключена!")
    exit()
photo_count = 0
while True:
    ret, frame = cap.read()
    if not ret: break
    cv2.imshow("Камера (Нажмите 'S' для снимка, 'Q' для выхода)", frame)
    key = cv2.waitKey(1000)
    if key == ord('s'):
        photo_count+=1
        filename = f'photo_{photo_count}.jpg'
        cv2.imwrite(filename, frame)
        print(f'Сохранено {filename}')
    elif key==ord('q'):
        break
        
    command = input("Введите команду: ")
    rb.send_to_arduino_and_get_response(command)
    
cap.release()
cv2.destroyAllWindows()