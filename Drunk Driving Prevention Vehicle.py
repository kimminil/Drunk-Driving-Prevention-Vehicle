import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
from hx711 import HX711
import serial
import pynmea2
from flask import Flask, jsonify

# Flask 앱 설정
app = Flask(__name__)

# GPIO 및 기타 하드웨어 설정
button_pin = 18
alcohol_sensor_pin = 23
led_pin = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(alcohol_sensor_pin, GPIO.IN)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)  # 추가 LED 출력 설정
GPIO.output(26, GPIO.LOW)  # 초기 LED 상태 설정

# 카메라 및 얼굴 인식 초기화
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
face_cascade = cv2.CascadeClassifier('/home/kkk/Desktop/haarcascade_frontalface_default.xml')

# 무게 센서 설정
hx = HX711(20, 16)
hx.set_reading_format("MSB", "MSB")
hx.set_reference_unit(114)
hx.reset()
hx.tare()

# 버튼 상태 확인 함수
def check_button():
    return GPIO.input(button_pin) == GPIO.LOW

# 얼굴 감지 및 무게 측정 함수
def detect_faces_and_measure_weight():
    with PiRGBArray(camera, size=(640, 480)) as rawCapture:
        face_detected_time = None
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            print(f"Detected {len(faces)} faces")
           
            if len(faces) == 1:
                if face_detected_time is None:
                    face_detected_time = time.time()
                elif time.time() - face_detected_time >= 3:
                    print("Single face continuously detected for 3 seconds.")
                    return get_weight()
            else:
                face_detected_time = None
           
            rawCapture.truncate(0)
           
    return 0

# 무게 측정 함수
def get_weight():
    val = hx.get_weight(5) + 500
    return max(0, val)

# 알코올 감지 함수
def check_alcohol():
    return GPIO.input(alcohol_sensor_pin) == 0

# GPS 데이터 수집 함수
def get_gps_data():
    ser = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.5)
    line = ser.readline().decode('utf-8', 'ignore').strip()
    if line.startswith('$GPRMC'):
        try:
            msg = pynmea2.parse(line)
            if msg.status == 'A':
                return {'latitude': msg.latitude, 'longitude': msg.longitude}
        except pynmea2.ParseError as e:
            print(f"GPS parse error: {e}")
    return {'latitude': None, 'longitude': None}

# Flask API 경로 설정
@app.route('/api/data', methods=['GET'])
def get_data():
    data = {
        'button_pressed': check_button(),
        'weight': detect_faces_and_measure_weight(),
        'alcohol_detected': check_alcohol(),
        'gps': get_gps_data()
    }
    return jsonify(data)

# 시스템 종료 전 정리 함수
def clean_and_exit():
    GPIO.cleanup()
    camera.close()
    print("System cleaned and exited.")

# Flask 서버 실행
if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)  # 외부 접속 가능하게 설정
    except KeyboardInterrupt:
        clean_and_exit()
