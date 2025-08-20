#HAND GESTURE CONTROLLING LED 
import cv2
import mediapipe as mp
import serial
import time

# Attempt to open serial connection safely
try:
    arduino = serial.Serial(port='COM9', baudrate=9600, timeout=1)
    time.sleep(2)  # Give Arduino time to reset
    print("Connected to Arduino on COM9")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    arduino = None

# Initialize Mediapipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_drawing = mp.solutions.drawing_utils

def detect_fingers(hand_landmarks):
    finger_tips = [8, 12, 16, 20]  # Index to Pinky
    thumb_tip = 4
    finger_states = [0, 0, 0, 0, 0]  # Thumb to Pinky

    # Check thumb (right hand assumption)
    if hand_landmarks.landmark[thumb_tip].x < hand_landmarks.landmark[thumb_tip - 1].x:
        finger_states[0] = 1

    # Check other fingers
    for idx, tip in enumerate(finger_tips):
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            finger_states[idx + 1] = 1

    return finger_states

# Start capturing video
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Failed to grab frame")
        break

    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    results = hands.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            fingers_state = detect_fingers(hand_landmarks)
            print(f"Fingers State: {fingers_state}")

            # Send to Arduino as 5-digit string (e.g., "10010\n")
            if arduino:
                try:
                    data = ''.join(map(str, fingers_state)) + '\n'
                    arduino.write(data.encode())
                except Exception as e:
                    print(f"Error writing to Arduino: {e}")

    cv2.imshow('Hand Tracking', image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()

if arduino:
    arduino.close()

