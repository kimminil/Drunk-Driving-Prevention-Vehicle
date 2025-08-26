import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
from hx711 import HX711
import serial
import pynmea2
import folium
import webbrowser

# GPIO 및 기타 하드웨어 설정
button_pin = 18
alcohol_sensor_pin = 23
led_pin = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(alcohol_sensor_pin, GPIO.IN)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)  # Set GPIO 26 as an output
GPIO.output(26, GPIO.LOW)  # Initialize GPIO 26 to LOW

mymap = folium.Map(location=[0, 0], zoom_start=12)
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
face_cascade = cv2.CascadeClassifier('/home/kkk/Desktop/haarcascade_frontalface_default.xml')
hx = HX711(20, 16)
hx.set_reading_format("MSB", "MSB")
hx.set_reference_unit(114)
hx.reset()
hx.tare()

def check_button():
    print("Checking button...")
    return GPIO.input(button_pin) == GPIO.LOW

def measure_weight():
    print("Measuring weight...")
    while True:
        val = hx.get_weight(5) + 500  # -1655 ~140
        print(f"Measured Weight: {val} grams")
        if val >= 0:
            print("Weight threshold exceeded, proceeding to face detection...")
            return detect_faces()  # 얼굴 감지로 넘어갑니다.
        hx.power_down()
        hx.power_up()
        time.sleep(0.5)
    return False  # Return False if weight threshold is never exceeded

def detect_faces():
    with PiRGBArray(camera, size=(640, 480)) as rawCapture:
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            image = frame.array
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            print(f"Detected {len(faces)} faces")
            for (x, y, w, h) in faces:
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.imshow("Face Detection", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rawCapture.truncate(0)
            if len(faces) == 1:
                print("Single face detected.")
                return check_alcohol()  # 알코올 검사로 넘어갑니다.
    cv2.destroyAllWindows()

def check_alcohol():
    print("Checking for alcohol...")
    last_state_change_time = time.time()
    continuous_state = None
    continuous_detected = False

    while True:
        current_time = time.time()
        current_state = GPIO.input(alcohol_sensor_pin) == 0

        if current_state != continuous_state:
            last_state_change_time = current_time
            continuous_state = current_state
            continuous_detected = False
            state_message = "Alcohol detected. Monitoring..." if current_state else "No alcohol detected, continuing to check..."
            print(state_message)

        if continuous_state and not continuous_detected and (current_time - last_state_change_time >= 10):
            continuous_detected = True
            print("Alcohol continuously detected for 10 seconds.")
            start_gps_time = current_time

        if continuous_detected and (current_time - start_gps_time <= 3):
            read_gps_data()

        if continuous_detected and (current_time - start_gps_time > 3):
            print("GPS data logging completed.")
            return True

        if not continuous_state and (current_time - last_state_change_time >= 10):
            print("No alcohol detected for 10 seconds. Starting countdown...")
            for i in range(3, 0, -1):
                print(f"{i}...")
                time.sleep(1)
            GPIO.output(26, GPIO.HIGH)
            print("Proceeding...")
            time.sleep(10)
            GPIO.output(26, GPIO.LOW)
            print("Operation complete.")
            return False

        time.sleep(0.5)

def read_gps_data():
    ser = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.5)
    line = ser.readline().decode('utf-8', 'ignore').strip()
    if line.startswith('$GPRMC'):
        try:
            msg = pynmea2.parse(line)
            if msg.status == 'A':
                print(f"Valid GPS Data - Latitude: {msg.latitude}, Longitude: {msg.longitude}")
                add_marker_to_map(msg.latitude, msg.longitude)
            else:
                print(f"Invalid GPS Data - Status: {msg.status}, Latitude: {msg.latitude}, Longitude: {msg.longitude}")
        except pynmea2.ParseError as e:
            print(f"Parse error: {e}")

def add_marker_to_map(latitude, longitude):
    m = folium.Map(location=[latitude, longitude], zoom_start=12)
    folium.Marker([latitude, longitude], popup='Current Location').add_to(mymap)
    map_file = '/home/kkk/Desktop/주소/map.html'  # 저장할 HTML 파일 경로
    mymap.save(map_file)
    webbrowser.open(map_file)

def cleanAndExit():
    print("Cleaning up...")
    GPIO.cleanup()
    camera.close()
    sys.exit()

def main():
    try:
        while True:
            if check_button():
                print("System activated.")
                if measure_weight():
                    print("System running normally.")
                else:
                    print("Resetting system...")
            time.sleep(0.1)
    except KeyboardInterrupt:
        cleanAndExit()

if __name__ == "__main__":
    main()
