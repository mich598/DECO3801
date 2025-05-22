import board
import neopixel
import time
import busio
import digitalio
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from gpiozero import DistanceSensor
import spidev
import adafruit_dht
from picamera2 import Picamera2, Preview
import os
import RPi.GPIO as GPIO

import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing import image
import cv2

import asyncio
import websockets
import json

import io
from io import BytesIO
import base64

headers = {
    "Cookie": "EAIT_WEB=Qmgtm4rpVjTJIWkxrjHXrtxcDbVCZEIZ",  # Example session cookie. Change when expires
    "Origin": "https://deco3801-adapted.uqcloud.net"
}

# Initialize Grow Lights (NeoPixels)
pixels = neopixel.NeoPixel(board.D18, 150, brightness=0.0, auto_write=False)
# Store current color to re-apply after brightness change
current_color = (0, 0, 0)

# Initialise TDS Sensor
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D8)  # CS on GPIO8
mcp = MCP.MCP3008(spi, cs)
chan = AnalogIn(mcp, MCP.P0)

# Initialise Ultrasonic Sensor. GPIO19 for trigger and GPIO26 for echo
# NOTE: Range is good up to 1m
uds = DistanceSensor(trigger=19, echo=26)

# Open SPI bus for pH Sensor
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device (CE0)
spi.max_speed_hz = 1350000

# Initialise Temperature and Humidity Sensor
# Use GPIO5 (physical pin 7)
dht_device = adafruit_dht.DHT11(board.D4)

# Pump Pins
DIR_A = 17     # Direction control
PWM_A = 27     # PWM speed control

DIR_B = 22     # Direction
PWM_B = 23     # PWM

# Nutrient Level
NUTRIENT_LEVEL = 400 #ppm

# Water Volume
MIN_VOL = 4 # L
MAX_VOL = 6 # L

BUCKET_LENGTH = 20 # cm
BUCKET_WIDTH = 20  # cm
BUCKET_VOL = 12    # L

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_A, GPIO.OUT)
GPIO.setup(PWM_A, GPIO.OUT)

GPIO.setup(DIR_B, GPIO.OUT)
GPIO.setup(PWM_B, GPIO.OUT)

# Initialize PWM on PWM_A pin at 1000Hz
pwm_a = GPIO.PWM(PWM_A, 1000)
pwm_a.start(0)

pwm_b = GPIO.PWM(PWM_B, 1000)
pwm_b.start(0)

def set_colour(color):
    global current_color
    current_color = color
    pixels.fill(color)
    pixels.show()

def set_brightness(brightness):
    # Change brightness (requires reinitializing)
    global pixels
    pixels.deinit()  # Clean up the previous instance
    pixels = neopixel.NeoPixel(board.D18, 150, brightness=brightness, auto_write=False)
    pixels.fill(current_color)
    pixels.show()

def voltage_to_tds(voltage):
    # Apply TDS conversion formula from DFRobot
    tds = (133.42 * voltage**3 - 255.86 * voltage**2 + 857.39 * voltage)
    return round(tds, 2)

# Read MCP3008 channel
def read_channel(channel):
    if channel < 0 or channel > 7:
        return -1
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    result = ((adc[1] & 3) << 8) + adc[2]
    return result

# Convert raw value to voltage (assuming VREF = 3.3V)
def convert_to_voltage(raw_val):
    return (raw_val * 3.3) / 1023.0

def voltage_to_ph(voltage):
    # Example: linear mapping, requires calibration
    # 0V ~ pH 0, 3.0V ~ pH 14
    return 7 + ((2.5 - voltage) / 0.18) + 3  # Typical 2.5V at pH 7

# Run motor in given direction indefinately
def run_motor(motor, direction=True):
    if motor == "motor_a":
        GPIO.output(DIR_A, GPIO.HIGH if direction else GPIO.LOW)
        pwm_a.ChangeDutyCycle(100)
    elif motor == "motor_b":
        GPIO.output(DIR_B, GPIO.HIGH if direction else GPIO.LOW)
        pwm_b.ChangeDutyCycle(100)

def stop_motor(motor, direction=True):
    if motor == "motor_a":
        pwm_a.ChangeDutyCycle(0)  # Stop motor
    elif motor == "motor_b":
        pwm_b.ChangeDutyCycle(0)  # Stop motor

def read_all_sensors(brightness):
    # pH Sensor
    raw_value = read_channel(1)  # CH1
    voltage = convert_to_voltage(raw_value)
    ph = voltage_to_ph(voltage)
    print(f"pH Level: {ph:.2f}")
    
    # Temperature and Humidity Sensor
    temperature = dht_device.temperature
    humidity = dht_device.humidity
    if temperature is not None:
        print(f"Temperature Level: {temperature:.1f}°C")
    if humidity is not None:
        print(f"Humidity Level: {humidity:.1f}%")

    # TDS Sensor
    voltage = chan.voltage
    tds = voltage_to_tds(voltage)
    print(f"Nutrient Level: {tds} ppm")

    # Ultrasonic Sensor
    height = uds.distance * 100
    volume = BUCKET_VOL - (BUCKET_LENGTH * BUCKET_WIDTH * height) / 1000
    print(f"Water Volume: {volume} L")
    
    # LED Grow Lights
    set_brightness(brightness)
    print(f"Brightness Level: {brightness}")

    return {
        "pH": round(ph,2),
        "temperature": temperature,
        "humidity": humidity,
        "tds": tds,
        "volume": round(volume, 2),
        "brightness": round(brightness, 1),
    }
        


async def async_main():
    
    uri = "wss://deco3801-adapted.uqcloud.net/ws/pi/plant1/"
    
    # LED Initialisation
    led_state = True
    
    # Initialise as Auto Mode
    light_auto = True
    water_auto = True
    nut_auto = True
    
    brightness = 0.1
    led_on = (255, 255, 255)
    led_off = (0, 0, 0)

    set_brightness(brightness)
    set_colour(led_on)
    
    # Camera Initialisation
    home_dir = os.environ['HOME']
    picam2 = Picamera2()
    config = picam2.create_still_configuration()
    picam2.configure(config)
    picam2.start_preview(Preview.NULL)
    picam2.start()
    
    # Get time when system starts
    start = time.perf_counter()

    async with websockets.connect(uri, additional_headers=headers) as ws:
        while True:  
            end = time.perf_counter()
            # Get elapsed time in seconds
            elapsed_time = round(end - start, 0)
            
            print(f"Elapsed time: {elapsed_time} s")
            # Change LED state after 12 hours (43200 seconds)
            if light_auto is True and elapsed_time % 43200 == 0:
                if led_state is False:
                    set_colour(led_off)
                elif led_state is True:
                    set_colour(led_on)
                led_state =  not led_state
                
            # Take a photo once per loop
            filename = f"{home_dir}/Desktop/plant.jpg"
            picam2.capture_file(filename)
            
            # plant health model prediction
            img_path = filename
            health_model = load_model('/home/mchen67/Desktop/decocode/health_classifier.h5')
            img = image.load_img(img_path, target_size=(224, 224))
            x = image.img_to_array(img)
            x = np.expand_dims(x, axis=0)
            score = health_model.predict(x)[0][0] 
            print(score)
            
            # growth stage model prediction
            stage_model = load_model('/home/mchen67/Desktop/decocode/growth_stage_classifier.h5')
            preds = stage_model.predict(x)
            stage = np.argmax(preds[0])
            score2 = preds[0][stage]
            stage+=1
            
            health_status = ""
            stage_status = -1
            
            # Predict health status of plants
            if score < 0.5:
                health_status = "healthy"
            else:
                health_status = "unhealthy"
            
            # Predict Growth Stage of Plant. 
            if stage == 1:
                stage_status = 0    # Germination Period
            elif stage == 2:
                stage_status = 1    # Seedling Period
            elif stage == 3:
                stage_status = 2    # Vegetative Period
            elif stage == 4:
                stage_status = 3    # Maturation Period
            
            print(stage_status)
            print(health_status)

            # Read All Sensors and AI Predictions            
            sensor_data = read_all_sensors(brightness)
            sensor_data["stage"] = stage_status
            sensor_data["health"] = health_status

            # Resize and Compress to send JSON packet to web socket
            image_comp = cv2.imread(img_path)
            resized = cv2.resize(image_comp, (1200, 560), interpolation=cv2.INTER_AREA)
            _, compressed= cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            encoded = base64.b64encode(compressed.tobytes()).decode('utf-8')
            
            sensor_data["image"] = f"data:image/jpeg;base64, {encoded}"

            # Send data through JSON to web socket            
            await ws.send(json.dumps({ "message": sensor_data }))
            
            try:
                data = await asyncio.wait_for(ws.recv(), timeout=2.0)
                print("Data is " + data)

                request = json.loads(data).get("message")

                # Determine whether user is in manual or auto mode.
                # Requests sent from user
                if request == "light_auto":
                    light_auto = True
                elif request == "light_man":
                    light_auto = False
                if request == "water_auto":
                    water_auto = True
                elif request == "water_man":
                    water_auto = False 
                if request == "nut_auto":
                    nut_auto = True
                elif request == "nut_man":
                    nut_auto = FalseR

                # Decision Tree Machine Learning Algorithm
                # Adjust LED brightness every 1.5hrs (5400 seconds)
                if light_auto is True:
                    if 0 <= elapsed_time < 5400:
                        brightness = 0.1
                    elif 5400 <= elapsed_time < 10800:
                        brightness = 0.2
                    elif 10800 <= elapsed_time < 16200:
                        brightness = 0.3
                    elif 16200 <= elapsed_time < 21600:
                        brightness = 0.4
                    elif 21600 <= elapsed_time < 27000:
                        brightness = 0.5
                    elif 27000 <= elapsed_time < 32400:
                        brightness = 0.4
                    elif 32400 <= elapsed_time < 37800:
                        brightness = 0.3
                    elif 37800 <= elapsed_time < 43200:
                        brightness = 0.2
                    set_brightness(brightness)
                else:
                    # Real Time Adjustment From User Section
                    # Controlling brightness value. Capped at 0.5 for safety
                    if request == "+" and brightness < 0.5:
                        brightness = round(brightness + 0.1, 1)
                    if request == "-" and brightness > 0:
                        brightness = round(brightness - 0.1, 1)
                    set_brightness(brightness)
                    
                    # Controlling light status (on/off)
                    if request == "on":
                        set_colour(led_on)
                    if request == "off":
                        set_colour(led_off)
                
                # TDS Sensor Control uses Pump B (nutrient level)
                if nut_auto is True:
                    tds_val = sensor_data["tds"]
                    if tds_val < NUTRIENT_LEVEL:
                        print("Running motor B")
                        run_motor("motor_b", direction=True)
                    elif tds_val > NUTRIENT_LEVEL:
                        print("Stopping motor B")
                        stop_motor("motor_b", direction=True)
                else:
                    stop_motor("motor_b", direction=True)
                    # Controlling Nutrients. Uses set dispense amount for safety
                    if request == "nut":
                        run_motor("motor_b", direction=True)
                        time.sleep(5)
                        stop_motor("motor_b", direction=True)

                # Nutrient Dispensor Control uses Pump A (Water level)                
                if water_auto is True:
                    cur_vol = sensor_data["volume"]
                    if cur_vol < MIN_VOL:
                        # Turn on Pumps
                        print("Running motor A")
                        run_motor("motor_a", direction=True)
                    elif cur_vol > MAX_VOL:
                        # Turn off Pumps
                        print("Stopping motor A")
                        stop_motor("motor_a", direction=True)
                else:
                    stop_motor("motor_a", direction=True)
                    # Controlling Water. Uses set dispense amount for safety
                    if request == "water":
                        run_motor("motor_a", direction=True)
                        time.sleep(5)
                        stop_motor("motor_a", direction=True)
                
            except asyncio.TimeoutError:
                pass
                
    # Close camera
    picam2.stop_preview()
    picam2.close()
    
    # Stop Motors
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
    

if __name__ == "__main__":
    asyncio.run(async_main())
