# -*- coding: utf-8 -*-
"""
--------------------------------------------------------------------------
Servo Control using a Potentiometer
--------------------------------------------------------------------------
License:   
Copyright 2019 - <Odnan Galvan>
Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
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
--------------------------------------------------------------------------
Control a servo using a potentiometer
  - Potentiometer connected to AIN0 (P1_19)
  - Servo Connected to PWM (P1_36)
When potentiometer is changed, this will change the corresponding servo location
--------------------------------------------------------------------------
Background:
  - https://adafruit-beaglebone-io-python.readthedocs.io/en/latest/ADC.html
  - https://learn.adafruit.com/controlling-a-servo-with-a-beaglebone-black/writing-
a-program
"""
# Libraries
import time
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
#------------------------------------------------------------------------

# Constants
ANALOG_INPUT = "P1_19"                      # AIN0
SERVO_OUTPUT = "P1_36"                      # PWM0 A
#------------------------------------------------------------------------

#Variables
debug                   = True                    #Helps keep track of servo
servo_duty_min          = 15                      #Changes spin direction below about 20
servo_duty_max          = 80                      # Stops if taken to 100
servo_pwm_frequency     = 50                      # Frequency in Hz (cannot go under 1)
servo_update_time       = .1                      # Time in seconds
# ------------------------------------------------------------------------

# Functions
def setup():
    """Set up the hardware components."""
    ADC.setup()
# End def

def set_servo_angle(adc_pin, servo_pin):
   
    servo_duty_span = servo_duty_max - servo_duty_min
    angle           = float(ADC.read(adc_pin))
    duty            = ((angle * servo_duty_span) + servo_duty_min) 
    if (debug):
        print("angle = {0}; duty = {1}".format(angle, duty))
    PWM.set_duty_cycle(servo_pin, duty)
# End def

def cleanup():
    """Set up the hardware components."""
    PWM.cleanup()
# end def

def run_servo():
    setup()
    print("Servo Control Program Start")

    try:
        while True:
            PWM.start(SERVO_OUTPUT, (100 - servo_duty_min), servo_pwm_frequency)
            """Uses the span of the servo duty: max - min duty"""
            set_servo_angle(ANALOG_INPUT, SERVO_OUTPUT)
            time.sleep(1)                       
            PWM.stop(SERVO_OUTPUT)
            time.sleep(servo_update_time)           #Not necessary to make computer work at max speed
            cleanup()
            
    except KeyboardInterrupt:
        PWM.stop(SERVO_OUTPUT)
    print("Servo Control Program Finished")
#end def

def servo_on():
    PWM.start(SERVO_OUTPUT, (100 - servo_duty_min), servo_pwm_frequency) 
    set_servo_angle(ANALOG_INPUT, SERVO_OUTPUT)
#end def

def servo_off():
    PWM.stop(SERVO_OUTPUT)
#end def
# ------------------------------------------------------------------------

# Main script
print("hello")                  #Not necessary, but nice if you're looking at in on a computer
if __name__ == '__main__':
    run_servo()
    
"""Notice that there is no cleanup here, that will be done seprately in the together.py file"""
