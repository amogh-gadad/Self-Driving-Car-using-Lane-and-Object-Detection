import cv2
import torch
import RPi.GPIO as GPIO
import time
import numpy as np

# ==== Motor Pins (L298N) ====
IN1, IN2 = 17, 18
IN3, IN4 = 22, 23
ENA, ENB = 25, 24

# ==== Ultrasonic Sensor Pins ====
TRIG_F, ECHO_F = 5, 6    # Front
TRIG_L, ECHO_L = 19, 26  # Left
TRIG_R, ECHO_R = 20, 21  # Right

# ==== Setup GPIO ====
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

motor_pins = [IN1, IN2, IN3, IN4, ENA, ENB]
GPIO.setup(motor_pins, GPIO.OUT)

ultrasonic_trig = [TRIG_F, TRIG_L, TRIG_R]
ultrasonic_echo = [ECHO_F, ECHO_L, ECHO_R]

GPIO.setup(ultrasonic_trig, GPIO.OUT)
GPIO.setup(ultrasonic_echo, GPIO.IN)

# PWM setup
pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)
pwm_a.start(0)
pwm_b.start(0)
SPEED = 40

# ==== Motor Control Functions ====
def move_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(SPEED)
    pwm_b.ChangeDutyCycle(SPEED)

def stop_motors():
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def turn_right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_a.ChangeDutyCycle(SPEED)
    pwm_b.ChangeDutyCycle(SPEED)

def turn_left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(SPEED)
    pwm_b.ChangeDutyCycle(SPEED)

# ==== Pulse Burst Functions ====
def dynamic_pulse_burst_forward(dist_front, bursts=10):
    if dist_front <= 10:
        on_time, off_time = 0.05, 0.25
    elif dist_front <= 13:
        on_time, off_time = 0.07, 0.2
    elif dist_front <= 17:
        on_time, off_time = 0.1, 0.15
    else:
        on_time, off_time = 0.12, 0.1

    for _ in range(bursts):
        move_forward()
        time.sleep(on_time)
        stop_motors()
        time.sleep(off_time)

def dynamic_pulse_burst_turn(direction, dist_front, bursts=5):
    if dist_front <= 10:
        on_time, off_time = 0.05, 0.25
    elif dist_front <= 13:
        on_time, off_time = 0.07, 0.2
    elif dist_front <= 17:
        on_time, off_time = 0.1, 0.15
    else:
        on_time, off_time = 0.12, 0.1

    for _ in range(bursts):
        if direction == 'left':
            turn_left()
        elif direction == 'right':
            turn_right()
        time.sleep(on_time)
        stop_motors()
        time.sleep(off_time)

# ==== Ultrasonic Distance Function ====
def get_distance(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.01)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    timeout = time.time() + 0.05
    while GPIO.input(echo) == 0:
        if time.time() > timeout:
            return 999
    pulse_start = time.time()

    while GPIO.input(echo) == 1:
        if time.time() > timeout:
            return 999
    pulse_end = time.time()

    distance = (pulse_end - pulse_start) * 17150
    return round(distance, 2)

# ==== YOLO Setup ====
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
model.eval()
torch.set_grad_enabled(False)

# ==== Lane Detection ====
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)

def draw_lines(img, lines):
    if lines is None:
        return
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)

def lane_detection(frame):
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    roi_vertices = np.array([[
        (0, height),
        (width // 2 - 50, height // 2 + 50),
        (width // 2 + 50, height // 2 + 50),
        (width, height)
    ]], dtype=np.int32)

    cropped_edges = region_of_interest(edges, roi_vertices)
    lines = cv2.HoughLinesP(cropped_edges, 1, np.pi / 180, 50, 40, 100)
    line_img = np.zeros_like(frame)
    draw_lines(line_img, lines)
    return cv2.addWeighted(frame, 0.8, line_img, 1, 1), lines

def lane_steering_decision(lines, width):
    if lines is None:
        return 'forward'
    left_lines = right_lines = 0
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1 + 1e-4)
            if slope < -0.3:
                left_lines += 1
            elif slope > 0.3:
                right_lines += 1
    if left_lines > right_lines:
        return 'right'
    elif right_lines > left_lines:
        return 'left'
    return 'forward'

# ==== Main Loop ====
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        dist_front = get_distance(TRIG_F, ECHO_F)
        dist_left = get_distance(TRIG_L, ECHO_L)
        dist_right = get_distance(TRIG_R, ECHO_R)
        print(f"Distances - Front: {dist_front}, Left: {dist_left}, Right: {dist_right}")

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = model(img_rgb)
        detections = results.xyxy[0]

        object_in_front = False
        for *box, conf, cls in detections:
            x1, y1, x2, y2 = map(int, box)
            box_center_x = (x1 + x2) // 2
            if frame.shape[1] // 3 < box_center_x < 2 * frame.shape[1] // 3 and conf > 0.5:
                object_in_front = True
            label = f'{model.names[int(cls)]} {conf:.2f}'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        frame_with_lanes, lines = lane_detection(frame)
        steering = lane_steering_decision(lines, frame.shape[1])

        if dist_front <= 35:
            print(f"Obstacle ahead at {dist_front} cm. Executing dynamic burst logic.")
            stop_motors()
            time.sleep(0.5)

            obstacle_left = dist_left <= 10
            obstacle_right = dist_right <= 10

            if obstacle_left and obstacle_right:
                print("Both sides blocked. Stopping.")
                stop_motors()
                time.sleep(1)
                continue
            elif obstacle_right:
                print("Right blocked. Turning left with burst.")
                dynamic_pulse_burst_turn('left', dist_front)
                dynamic_pulse_burst_forward(dist_front)
                dynamic_pulse_burst_turn('right', dist_front)
            elif obstacle_left:
                print("Left blocked. Turning right with burst.")
                dynamic_pulse_burst_turn('right', dist_front)
                dynamic_pulse_burst_forward(dist_front)
                dynamic_pulse_burst_turn('left', dist_front)
            else:
                print("No side obstacles. Defaulting to right burst.")
                dynamic_pulse_burst_turn('right', dist_front)
                dynamic_pulse_burst_forward(dist_front)
                dynamic_pulse_burst_turn('left', dist_front)

            move_forward()
            continue

        elif object_in_front:
            print("Object detected ahead by YOLO, stopping.")
            stop_motors()
            time.sleep(0.1)
            continue

        if steering == 'left':
            print("Steering left.")
            turn_left()
            time.sleep(0.3)
            move_forward()
        elif steering == 'right':
            print("Steering right.")
            turn_right()
            time.sleep(0.3)
            move_forward()
        else:
            move_forward()

        cv2.imshow('Autonomous Bot View', frame_with_lanes)
        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    print("Cleaning up GPIO...")
    stop_motors()
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
