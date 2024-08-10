#Disclaimer : You can see this code really just me using ChatGPT and my struggle of using my little code understanding to do this LOL
#I just ask ChatGPT, then modify the code myself to fit in. Several debugging until my head blown.
#This code using camera on your computer(if you want to use on pc hence why I commenting that part) and made the camera can detect your face, presenting the distance and so on.
#It is meant to use with Raspi4 and Distance sensor VL530x, so you can just remove that part of the code if you don't want it.
#I also using tkinter to made desktop app as UI, so there will be folder saved and some func user can use.
#I will review back to this code using my current understanding, busy rn... till later. see ya ! (10/8/2024)

import threading
import cv2
import time
import os
import dlib
import numpy as np
import time
# import board
# import busio
# import adafruit_vl53l0x
import tkinter as tk
from tkinter import ttk
from tkinter import Checkbutton
from tkinter import filedialog

#Set up VL53L0X
#Initialize I2C communication
# i2c = busio.I2C(board.SCL, board.SDA)
# # Create a VL53L0X object
# vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# Set up the camera
camera = cv2.VideoCapture(0)

#pixels to mm
conversion_factor = 25.4/96

# Initialize face detector and predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

#Disable the tickbox
toggle = True
toggle1 = True
toggle2 = True
#Define flag
running = True
# Create the face detection event(flag)
face_detection_event = threading.Event()

#mapping resolution
resolution_max_distance_mapping = {
    (640, 480): 200,
    (800, 600): 240,
    (1280, 720): 290,
    (1600, 900): 300,
    (1920, 1080): 360
}
# Create a function to set camera settings to default values
max_distance = 200
resolution = (640, 480)
frame_rate = 10
camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
camera.set(cv2.CAP_PROP_FPS, frame_rate)

#Threadinglock
camera_lock = threading.Lock()

# Function to run the OpenCV window loop
def opencv_loop():
    global camera, frame
    while running:
        with camera_lock:
            ret, frame = camera.read()
            if not ret:
                break
            print(f"Current Resolution is {resolution}")
            print(f"Current Frame Rate is {frame_rate}")
            print(f"Current Max Distance is {max_distance}")
            detect_face()
            # Display the frame
            cv2.imshow('Camera View', frame)
            
            # Check for key press events
            key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:  # Check for 'q' or 'Esc' key
            close_opencv()
            root.quit()
            break

# Create a function for face detection
def detect_face():
    if toggle:
        while True:
            ret, frame = camera.read()  # Read frame from webcam
            if not ret:
                break
            #Convert frame to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect faces in the grayscale image
            faces = detector(gray)

            # Loop through all detected faces
            for face in faces:
                # Get the facial landmarks of the face
                landmarks = predictor(gray, face)

                # Extract the coordinates of the nose tip and mouth corner points
                nose_tip_x, nose_tip_y = landmarks.part(30).x, landmarks.part(30).y
                mouth_corner_x, mouth_corner_y = landmarks.part(54).x, landmarks.part(54).y
                left_eye_center_x, left_eye_center_y = landmarks.part(36).x, landmarks.part(36).y
                right_eye_center_x, right_eye_center_y = landmarks.part(45).x, landmarks.part(45).y
                chin_x, chin_y = landmarks.part(8).x, landmarks.part(8).y

                # Compute the distance between the nose tip and mouth corner points
                # dist_sensor = VL53L0X_reading()
                dist_sensor = distance_apparent(nose_tip_x, nose_tip_y, mouth_corner_x, mouth_corner_y, max_distance) *conversion_factor
                dist_apparent = distance_apparent(nose_tip_x, nose_tip_y, mouth_corner_x, mouth_corner_y, max_distance) *conversion_factor
                # Display the distance between the detected face and the camera
                if toggle1:
                    cv2.putText(frame, "{:.2f} mm".format(dist_sensor), (face.left(), face.bottom() + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # cv2.putText(frame, "{:.2f} mm".format(dist_apparent), (face.left(), face.bottom() + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                if toggle2:
                    cv2.putText(frame, "{:.2f} mm".format(dist_apparent), (face.left() +150, face.bottom() + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # If distance is less than 200 pixels, label facial landmarks
                if dist_apparent < 200:
                    # Draw rectangle around face
                    cv2.rectangle(frame, (face.left(), face.top()), (face.right(), face.bottom()), (0, 255, 0), 2)

                    # Label face
                    cv2.putText(frame, "face", (face.left(), face.top() - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Label nose
                    cv2.putText(frame, "nose", (nose_tip_x, nose_tip_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Label mouth
                    cv2.putText(frame, "mouth", (mouth_corner_x, mouth_corner_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            hasil = cv2.imshow('Camera View', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # Check for 'q' or 'Esc' key
                close_opencv()
                root.quit()
                break
            if not toggle:
                print(f"Face Detector {toggle}")
                break
        return hasil
    return toggle

# Create a function to change camera settings
def change_settings():
    global camera, resolution, frame_rate, opencv_thread, running, frame, max_distance
    resolution = resolution_values[resolution_combo.current()]
    frame_rate = int(frame_rate_combo.get())
    running = False
    toggle_detect_face()
    opencv_thread.join()
    close_opencv()
    max_distance = resolution_max_distance_mapping.get(resolution)
    with camera_lock:
        running = True
        toggle_detect_face() 
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        camera.set(cv2.CAP_PROP_FPS, frame_rate)
        opencv_thread = threading.Thread(target=opencv_loop)
        opencv_thread.start()

# Create a function to capture an image
def capture_image():
    ret, frame = camera.read()
    if ret:
        image_filename = os.path.join(output_folder, f'captured_image_{time.strftime("%Y%m%d-%H%M%S")}.jpg')
        cv2.imwrite(image_filename, frame)
        print(f"Captured image saved as {image_filename}")

# Define a function to compute distance between two points
def distance_apparent(x1, y1, x2, y2, max_distance):
    dist = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    inverted_dist = max_distance - dist
    return inverted_dist

# def VL53L0X_reading():
#     while True:
#         # Start measurement
#         vl53.measurement_timing_budget = 30000  # Set measurement timing budget (in microseconds)
#         vl53.start_continuous()

#         # Get distance measurement in millimeters
#         distance_mm = vl53.range

#         # Stop continuous measurement
#         vl53.stop_continuous()

#         # Print the distance measurement
#         print(f"Distance: {distance_mm} mm")

#         # Delay before next measurement
#         time.sleep(0.5)

#         if not toggle1:
#             vl53.close()

# Define the output folder
output_folder = ""
def set_output_folder():
    global output_folder
    chosen_folder = filedialog.askdirectory()  # Open the folder selection dialog
    if chosen_folder:
        output_folder = chosen_folder
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
      
def toggle_detect_face():
    global toggle
    toggle = not toggle


def toggle_sensor_reading():
    global toggle1
    toggle1 = not toggle1

def toggle_apparent_reading():
    global toggle2
    toggle2 = not toggle2

def close_opencv():
    # Release the camera and video writer
    camera.release()

    # Close all OpenCV windows
    cv2.destroyAllWindows()

def create_gui():
    global root, frame_rate_combo, resolution_values, resolution_combo, set_folder_button, capture_button, detect_face_checkbox, opencv_thread

    # Create a GUI window
    root = tk.Tk()
    root.title("Camera Interface")

    # Create buttons for frame rate selection
    frame_rate_label = tk.Label(root, text="Select Frame Rate:")
    frame_rate_label.pack()
    frame_rate_values = [10, 15, 20, 25, 30]
    frame_rate_combo = ttk.Combobox(root, values=frame_rate_values)
    frame_rate_combo.pack()

    # Create buttons for resolution selection
    resolution_label = tk.Label(root, text="Select Resolution:")
    resolution_label.pack()
    resolution_values = [(640, 480), (800, 600), (1280, 720), (1600, 900), (1920, 1080)]
    resolution_combo = ttk.Combobox(root, values=[f"{resolution[0]}x{resolution[1]}" for resolution in resolution_values])
    resolution_combo.pack()

    # Create a button to apply settings
    apply_button = tk.Button(root, text="Apply Settings", command=change_settings)
    apply_button.pack(pady=2)

    # Create a frame for output folder selection
    set_folder_frame = tk.Frame(root)
    set_folder_frame.pack(fill=tk.X, padx=10, pady=10)
    # Create a button to set the output folder
    set_folder_label = tk.Label(set_folder_frame, text="Select Output Folder:")
    set_folder_label.pack(side=tk.LEFT, padx=(70,0))
    set_folder_button = tk.Button(set_folder_frame, text="  ...  ", command=set_output_folder)
    set_folder_button.pack(side=tk.LEFT, padx=0)

    # Create a button to capture an image
    capture_button = tk.Button(root, text="Capture Image", command=capture_image)
    capture_button.pack(pady=10)

    # Create a checkbox for face detection
    detect_face_checkbox = Checkbutton(root, text="Toggle Detect Face", command=toggle_detect_face)
    detect_face_checkbox.pack()

    # Create a checkbox for sensor reading
    detect_face_checkbox = Checkbutton(root, text="Sensor Reading", command=toggle_sensor_reading)
    detect_face_checkbox.pack()

    # Create a checkbox for apparent reading
    detect_face_checkbox = Checkbutton(root, text="Apparent Reading", command=toggle_apparent_reading)
    detect_face_checkbox.pack()

    # Create a thread for the OpenCV window loop and detect face loop
    opencv_thread = threading.Thread(target=opencv_loop)
    opencv_thread.start()

    # Start the GUI main loop
    root.geometry("320x280")  # Set the window size pixels
    root.mainloop()

create_gui()