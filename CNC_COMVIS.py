"""
PROJECT: CNC Arm Computer Vision Controller
AUTHOR: Jorge Gael Santiago Simón
DESCRIPTION:
  Detects hand gestures using MediaPipe and sends mapped coordinates
  to an Arduino-based robot arm via Serial USB.
  
DEPENDENCIES:
  pip install opencv-python mediapipe pyserial
"""

import cv2
import mediapipe as mp
import serial
import time
import math

# --- CONFIGURATION ---
SERIAL_PORT = 'COM5'   # Change this to your Arduino port (e.g., 'COM3' or '/dev/ttyUSB0')
BAUD_RATE   = 9600
CAMERA_ID   = 0        # Default webcam index
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- INITIALIZATION ---

# 1. Serial Connection
arduino = None
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) # Allow Arduino to reset
    print(f"✅ Successfully connected to {SERIAL_PORT}")
except Exception as e:
    print(f"⚠️ Warning: Serial connection failed. Running in Simulation Mode. ({e})")

# 2. MediaPipe Hands Setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)
mp_draw = mp.solutions.drawing_utils

# 3. Camera Setup
cap = cv2.VideoCapture(CAMERA_ID)
cap.set(3, FRAME_WIDTH)
cap.set(4, FRAME_HEIGHT)

print("🚀 Controller Running. Move your index finger to guide the robot.")
print("Press 'q' to exit.")

last_send_time = 0
SEND_INTERVAL = 0.05 # Limit serial transmission rate (20Hz)

# --- MAIN LOOP ---
while True:
    success, img = cap.read()
    if not success:
        print("❌ Camera error.")
        break
    
    # Flip image for intuitive mirror interaction
    img = cv2.flip(img, 1)
    
    # Convert BGR to RGB for MediaPipe processing
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    
    if results.multi_hand_landmarks:
        for hand_lms in results.multi_hand_landmarks:
            # Draw skeleton
            mp_draw.draw_landmarks(img, hand_lms, mp_hands.HAND_CONNECTIONS)
            
            # --- FEATURE EXTRACTION ---
            # Landmark 8: Index Finger Tip
            idx_pt = hand_lms.landmark[8]
            
            # Convert normalized coordinates to pixel space
            cx, cy = int(idx_pt.x * FRAME_WIDTH), int(idx_pt.y * FRAME_HEIGHT)
            
            # Visual feedback (Cursor)
            cv2.circle(img, (cx, cy), 15, (255, 0, 255), cv2.FILLED)
            
            # --- COORDINATE MAPPING ---
            # Map screen space (Pixels) to Robot Control Space (0-100)
            # We define a "Virtual Control Box" in the center of the screen
            margin = 100
            
            # Normalize X (0-100)
            target_x = int( ( (cx - margin) / (FRAME_WIDTH - 2*margin) ) * 100 )
            
            # Normalize Y (0-100) - Inverted because camera Y=0 is top
            target_y = int( 100 - ( (cy - margin) / (FRAME_HEIGHT - 2*margin) ) * 100 )
            
            # Clamp values to safe range
            target_x = max(0, min(100, target_x))
            target_y = max(0, min(100, target_y))
            
            # Display data on HUD
            cv2.putText(img, f"CMD: X{target_x} Y{target_y}", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Draw Control Box boundary
            cv2.rectangle(img, (margin, margin), 
                          (FRAME_WIDTH-margin, FRAME_HEIGHT-margin), (0, 255, 255), 2)

            # --- SERIAL TRANSMISSION ---
            current_time = time.time()
            if current_time - last_send_time > SEND_INTERVAL:
                if arduino and arduino.is_open:
                    # Protocol: "X{val}Y{val}"
                    command = f"X{target_x}Y{target_y}"
                    arduino.write(command.encode())
                
                last_send_time = current_time

    # Render
    cv2.imshow("CNC Hand Controller", img)
    
    # Exit condition
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()