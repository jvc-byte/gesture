import cv2
import mediapipe as mp
import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

# Initialize Mediapipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7,
                       min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

# Function to detect if hand is open or closed


def detect_hand_state(hand_landmarks):
    # Finger tips landmarks
    finger_tips = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky
    # Lower finger joints for comparison
    # Thumb base, Index MCP, Middle MCP, Ring MCP, Pinky MCP
    finger_bases = [2, 5, 9, 13, 17]

    # Count extended fingers
    extended_fingers = 0

    # Check if thumb is extended (different comparison for thumb)
    if hand_landmarks.landmark[finger_tips[0]].x < hand_landmarks.landmark[finger_bases[0]].x:
        extended_fingers += 1

    # Check other fingers by comparing y-coordinates
    for i in range(1, 5):
        if hand_landmarks.landmark[finger_tips[i]].y < hand_landmarks.landmark[finger_bases[i]].y:
            extended_fingers += 1

    # If at least 3 fingers are extended, consider the hand open
    is_open = extended_fingers >= 3

    return is_open, extended_fingers


# Start capturing video
cap = cv2.VideoCapture(0)

# Variables to track state and implement debouncing
current_state = None  # None, True (open), False (closed)
state_counter = 0
DEBOUNCE_FRAMES = 5  # Number of consistent frames needed to change state
last_servo_angle = None

while cap.isOpened():
    success, image = cap.read()
    if not success:
        break

    # Convert image for processing
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    image.flags.writeable = False
    results = hands.process(image)
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    hand_detected = False
    # Process hand landmarks
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            hand_detected = True
            mp_drawing.draw_landmarks(
                image, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                mp_drawing.DrawingSpec(
                    color=(0, 255, 0), thickness=2, circle_radius=4),
                mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2)
            )

            is_open, extended_fingers = detect_hand_state(hand_landmarks)

            # Debouncing logic
            if current_state is None:
                current_state = is_open
                state_counter = 1
            elif current_state == is_open:
                state_counter += 1
            else:
                state_counter -= 1
                if state_counter <= 0:
                    current_state = is_open
                    state_counter = 1

            # Calculate servo angle based on hand state
            if state_counter >= DEBOUNCE_FRAMES:
                servo_angle = 0 if is_open else 90  # 90° when open, 0° when closed

                # Only send command if the angle changed
                if last_servo_angle != servo_angle:
                    arduino.write(bytes([servo_angle]))
                    print(f"Servo Angle Sent: {servo_angle}°")
                    last_servo_angle = servo_angle

            # Display info on screen
            hand_state = "OPEN" if is_open else "CLOSED"
            cv2.putText(image, f"Door: {hand_state}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(image, f"Fingers Up: {extended_fingers}/5", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(image, f"Servo: {last_servo_angle if last_servo_angle is not None else 'N/A'}°", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # If no hand detected for some frames, reset the state
    if not hand_detected:
        current_state = None
        state_counter = 0

    # Display the image
    cv2.imshow('Hand Gesture Access Control', image)
    if cv2.waitKey(5) & 0xFF == 27:  # ESC key to exit
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()
