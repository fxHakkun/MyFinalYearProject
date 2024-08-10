#This is for my sensortest code of VL530x or whatsoever. 
#yeah.. all this still using chatgpt(cause it's easy)
#but doesnt mean I rely on it 100% ... maybe just 90% and 10% crushing my head. LOL

#I actually have confindence on doing this project cause I know I can understand the logic and algorithm just well, I just lack basic. Until today(10/8/2024)

import time
import board
import busio
import adafruit_vl53l0x

# Initialize I2C communication
i2c = busio.I2C(board.SCL, board.SDA)

# Create a VL53L0X object
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

while True:
    # Start measurement
    vl53.measurement_timing_budget = 20000  # Set measurement timing budget (in microseconds)
    vl53.start_continuous()

    # Get distance measurement in millimeters
    distance_mm = vl53.range

    # Stop continuous measurement
    vl53.stop_continuous()

    # Print the distance measurement
    print(f"Distance: {distance_mm} mm")

    # Delay before next measurement
    time.sleep(0.5)
