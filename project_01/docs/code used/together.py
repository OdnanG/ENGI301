# -*- coding: utf-8 -*-
"""
License:   
Copyright 2019 Odnan Galvan

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
# Libraries
import time
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
import board
import busio
import adafruit_tsl2561
import os
import servo
import clock_1
#-----------------------------------------------------------------------------

# Main Script (Light Sensor)
"""This first part is dedicated to get the light sensor working
code can be found on the adafruit website for TSL2561"""
i2c2 = busio.I2C(board.SCL_2, board.SDA_2)
sensor = adafruit_tsl2561.TSL2561(i2c2)

print('Lux: {}'.format(sensor.lux))
print('Broadband: {}'.format(sensor.broadband))
print('Infrared: {}'.format(sensor.infrared))
print('Luminosity: {}'.format(sensor.luminosity))
#-----------------------------------------------------------------------------

# Main Script 
"""This starts and stops both the servo and the LCD display
depending on the light sensor output"""
servo.setup()
servo_on = False
try:
    while True:
        infra = format(sensor.infrared)
        if float(infra) > 1000:
            if not servo_on:
                servo.servo_on()
                servo_on = True
            
            clock_1.clock_on()
            
        else: 
            if servo_on:
                servo.servo_off()
                servo_on = False
                
            clock_1.clock_off()
            
        time.sleep(.3)  
        
except KeyboardInterrupt:
    servo.servo_off()
    clock_1.clock_off()

    
servo.cleanup()         #Cleanup for servo is done here instead of in servo.py
            
