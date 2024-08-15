from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import serial
import time
import math

from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import time

tx_rx_data = 0x00

cup_class_id      = 0
entrance_class_id = 1
holder_class_id   = 2
straw_class_id    = 3

PY_RESET          = 0
PY_READY          = 1
PY_START          = 2
PY_DETECTING      = 3
PY_DETECTED       = 4

current_state = PY_RESET

ONLY_WAITING                = 0
RESET                       = 1
RESET_QUEUE                 = 2 
RESET_SERVO                 = 3
RESET_HORIZONTAL_INTERRUPT  = 4 
RESET_HORIZONTAL_DEFAULT    = 5 
RESET_VERTICAL_DEFAULT      = 6
STARTING                    = 7
VERTICAL_HOLD_CUP           = 8
VERTICAL_REMOVE_LID_HOLDER  = 9
HORIZONTAL_HOLD_HOLDER      = 10
HORIZONTAL_PLACE_HOLDER     = 11
HORIZONTAL_HOLD_CUP         = 12
HORIZONTAL_WASTE_CUP        = 13
HORIZONTAL_WASH_CUP         = 14
HORIZONTAL_LOAD_CUP         = 15

model = None

py_serial = serial.Serial(
    port = "/dev/ttyCH341USB0",
    baudrate = 9600
)

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins = '*')

class Cup:
    def __init__(self, x1, y1, x2, y2, width, height):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.width = width
        self.height = height

class Entrance:
    def __init__(self, x1, y1, x2, y2, width, height):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.width = width
        self.height = height

class Holder:
    def __init__(self, x1, y1, x2, y2, width, height):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.width = width
        self.height = height

model_path = "../0812/best.pt"
webcam = None

# 0 : holder, 1 ~ 2 : 컵 크기에 따른 size, 3 ~ 4 : entrance 크기에 따른 size

# 00 : None
# 01 : small
# 10 : regular
# 11 : large
def results_processing(largest_cup, largest_entrance, largest_holder):
    commend = 0x00

    # 아예 인식되지 않은 경우는 바로 리턴
    if largest_cup is None or largest_entrance is None:
        print("there's no cup")
        return commend
    
    # 인식됐지만 유효하지 않은 경우도 리턴
    # 추후 수정

    if largest_holder:
        print("w. Holder ")
        # size 잘 맞으면 홀더로 인식 // 
        commend = commend | 0x01
    else:
        print("wo. Holder")

    commend = commend << 2

    # 컵 크기에 따른 size
    entrance_2_cup = largest_cup.y2 - largest_entrance.y2
    if entrance_2_cup >= 420:
        print("Cup size : Large")
        commend = commend | 0x03
    elif entrance_2_cup < 420 and entrance_2_cup > 350:
        print("Cup size : Regular")
        commend = commend | 0x02
    else:
        print("Cup size : Small")
        commend = commend | 0x01

    commend = commend << 2

    # entrance 크기에 따른 size
    entrance_x_diff = largest_entrance.x2 - largest_entrance.x1
    if entrance_x_diff >= 280:
        print("Ent size : Large")
        commend = commend | 0x03
    elif entrance_x_diff < 280 and entrance_x_diff > 245:
        print("Ent size : Regular")
        commend = commend | 0x02
    else:
        print("Ent size : Small")
        commend = commend | 0x01

    commend = commend << 3
    return commend

    # 인식된 컵 없는 경우 'F' 리턴
    # 인식된 컵이 있는 경우 


def state_update():
    global RESET, READY, START, DETECTING, DETECTED, model_path, webcam, current_state, model
    global tx_rx_data
    tx_rx_data = 0x00
    
    response = None
    print(current_state)

    if py_serial.readable():
        
        if current_state == PY_RESET:
            time.sleep(5)
            model = YOLO(model_path)
            print("model loaded")
            current_state = PY_READY
            return
        
        elif current_state == PY_READY:
            while ((tx_rx_data >> 7) & 0x01) == 0:
                
                tx_rx_data = py_serial.readline()
                tx_rx_data = int(tx_rx_data.decode()[:-2])

            current_state = PY_START
            return

        elif current_state == PY_START:
            webcam = cv2.VideoCapture(2)
            while (webcam.isOpened()) == None:
                webcam = cv2.VideoCapture(2)
                time.sleep(0.1)

            print("Webcam is opened")
            current_state = PY_DETECTING
            return

        elif current_state == PY_DETECTING:
            ret, frame = webcam.read()
            while ret is False:
                print("Capture video")
                ret, frame = webcam.read()

            largest_cup = None
            largest_entrance = None
            largest_holder = None

            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

            results = model(frame,imgsz = (640,640))

            for result in results:
                for box in result.boxes:
                    confidence = math.ceil((box.conf[0]*100))/100
                        
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    width = x2 - x1
                    height = y2 - y1
                    print(confidence, int(box.cls[0]), width, height)
                    if confidence < 0.6 or width > 350 or width < 200: 
                        continue

                    object_size = width * height
                    class_id = int(box.cls[0])

                    if class_id == cup_class_id:
                        if largest_cup is None or object_size > (largest_cup.width * largest_cup.height):
                            largest_cup = Cup(x1, y1, x2, y2, width, height)

                    elif class_id == entrance_class_id:
                        if largest_entrance is None or object_size > (largest_entrance.width * largest_entrance.height):
                            largest_entrance = Entrance(x1, y1, x2, y2, width, height)
                                
                    elif class_id == holder_class_id:
                        if largest_holder is None or object_size > (largest_holder.width * largest_holder.height):
                            largest_holder = Holder(x1, y1, x2, y2, width, height)


            commend = results_processing(largest_cup, largest_entrance, largest_holder) #??
            print("commend", commend)

            socketio.emit('cup update', {'cup' : commend})
            py_serial.write(commend.to_bytes(1, byteorder='big'))
            time.sleep(0.1)
            current_state = PY_DETECTED
            return

        elif current_state == PY_DETECTED:
            webcam.release()

            while True:
                response = py_serial.readline()
                while response is None:
                    response = py_serial.readline()

                response = int(response.decode()[:-2])

                # 현재 아두이노에서 처리중인 컵이 없는 경우
                if response & (1 << 7) : 
                    current_state = PY_READY
                    return

                response = response >> 3

                if response == VERTICAL_HOLD_CUP :
                    socketio.emit('state update', {'state' : 1})
                elif response == HORIZONTAL_WASTE_CUP:
                    socketio.emit('state update', {'state' : 2})
                elif response == HORIZONTAL_LOAD_CUP:
                    socketio.emit('state update', {'state' : 3})

                time.sleep(0.1)

                print(response)
                if response == HORIZONTAL_LOAD_CUP:
                    time.sleep(0.5)
                    current_state = PY_READY
                    return

    else:
        print("Serial disconnected")
        return
    
@app.route('/')
def index():
    return render_template('UI_test.html')

def start_state_update_loop():
    while True:
        state_update()

if __name__ == "__main__":
    socketio.start_background_task(target=start_state_update_loop)
    socketio.run(app, host = 'localhost', port = 5113, debug = True)