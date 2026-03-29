import cv2
import numpy as np
import serial 
import time

COM_PORT = 'COM4'  
BAUD_RATE = 115200 

arduino = None
try:
    arduino = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) 
    print(f"Connected to TIVA on {COM_PORT}")
except:
    print("Running in VISION ONLY mode")

url = "http://10.61.180.2:4747/video" 
cap = cv2.VideoCapture(url)
if not cap.isOpened(): exit()

ret, frame = cap.read()
FRAME_WIDTH = frame.shape[1] if ret else 640
FRAME_HEIGHT = frame.shape[0] if ret else 480

lower_color = np.array([0, 112, 163])
upper_color = np.array([179, 255, 255])

target_x = FRAME_WIDTH // 2
target_y = FRAME_HEIGHT // 2

# ---PID GAINS---
Kp = 15  
Ki = 0.2
Kd = 20 

ERROR_THRESHOLD = 50
MIN_TILT_OFFSET = 5

# State Variables
prev_error_x = 0
prev_error_y = 0
integral_x = 0
integral_y = 0

def send_command(x_err, y_err):
    global prev_error_x, prev_error_y, integral_x, integral_y
    p_x = x_err * Kp
    p_y = y_err * Kp
    d_x = (x_err - prev_error_x) * Kd
    d_y = (y_err - prev_error_y) * Kd
    integral_x += x_err
    integral_y += y_err
    integral_x = np.clip(integral_x, -1000, 1000)
    integral_y = np.clip(integral_y, -1000, 1000)
    
    i_x = integral_x * Ki
    i_y = integral_y * Ki
    
    prev_error_x = x_err
    prev_error_y = y_err

    control_x = p_x + i_x + d_x
    control_y = p_y + i_y + d_y
    
    offset_x = control_x / 3.0
    offset_y = control_y / 3.0

    # --- X-AXIS LOGIC ---
    if abs(x_err) < ERROR_THRESHOLD:
        angle_x = 90
        integral_x = 0 # Reset patience when goal reached
    else:
        if abs(offset_x) < MIN_TILT_OFFSET:
            offset_x = MIN_TILT_OFFSET * np.sign(offset_x)
        angle_x = int(90 + offset_x)

    # --- Y-AXIS LOGIC ---
    if abs(y_err) < ERROR_THRESHOLD:
        angle_y = 90
        integral_y = 0 
    else:
        if abs(offset_y) < MIN_TILT_OFFSET:
            offset_y = MIN_TILT_OFFSET * np.sign(offset_y)
        angle_y = int(90 + offset_y)
    
   
    angle_x = np.clip(angle_x, 45, 135)
    angle_y = np.clip(angle_y, 45, 135)
    
    cmd = f"X:{angle_x},Y:{angle_y}\n"
    # print(f"Err: {x_err} I-Term: {i_x:.1f} Cmd: {angle_x}") 

    if arduino and arduino.is_open:
        try:
            arduino.write(cmd.encode('utf-8'))
        except:
            pass

while True:
    ret, frame = cap.read()
    if not ret: break
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.dilate(mask, None, iterations=2) 
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    
    cv2.rectangle(frame, (target_x - ERROR_THRESHOLD, target_y - ERROR_THRESHOLD),
                  (target_x + ERROR_THRESHOLD, target_y + ERROR_THRESHOLD), (0, 255, 0), 2)

    if len(contours) > 0:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 100: 
            M = cv2.moments(largest)
            if M["m00"] != 0:
                ball_x = int(M["m10"] / M["m00"])
                ball_y = int(M["m01"] / M["m00"])

                error_x = target_x - ball_x
                error_y = target_y - ball_y
                
                send_command(error_x, error_y)

              
                color = (0, 255, 255)
                if abs(error_x) > ERROR_THRESHOLD or abs(error_y) > ERROR_THRESHOLD:
                    color = (0, 0, 255)
                
                cv2.circle(frame, (ball_x, ball_y), 10, color, 2)
                cv2.line(frame, (target_x, target_y), (ball_x, ball_y), color, 2)
                
                
                cv2.putText(frame, f"Err: {error_x},{error_y}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    cv2.imshow('Ball Stabilizer', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
if arduino: arduino.close()
