# DECO3801 GREENSYNC RASPBERRY PI CODE
This code performs sensor readings on the temperature and humidity sensors, TDS sensor, pH sensor, ultrasonic sensor, water dispenser setup, nutrient dispenser setup, light setup and camera setup. Images from camera are passed into the plant health CNN model and growth status CNN model for predictive analysis. Prediction results and sensor readings are passed into JSON packet which are transmitted to the dashboard via web socket.  

## Before Running Code
Ensure Cookies are updated to the correct version

Fill reservoir to 5L minimum

Check reservoir nutrient level is above 400 ppm

NOTE: Nutrient Levels and Min/Max Water Volume are adjustable according to the plant type and reservoir size.

## Running Code Instructions
1. Open Raspberry Pi terminal
2. Activate the virtual environment by typing the following command:

source wsenv/bin/activate

3. Run the program from the virtual environment using: 
    
sudo -E ./wsenv/bin/python /home/mchen67/Desktop/main.py
