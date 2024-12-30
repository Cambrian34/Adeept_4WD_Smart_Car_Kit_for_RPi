#!/usr/bin/env python
from __future__ import division
import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685

colors = [0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0X6F00D2, 0xFF5809]

# REG1: 15,16,18 / RGB2: 19,21,22
R = 15
G = 16
B = 18

pwm = Adafruit_PCA9685.PCA9685()

dir_mid = 425
dir_ch = 0

Motor_A_EN    = 7
Motor_B_EN    = 11

Motor_A_Pin1  = 8
Motor_A_Pin2  = 10
Motor_B_Pin1  = 13
Motor_B_Pin2  = 12

Dir_forward   =  0
Dir_backward  =  1

pwm_A = 0
pwm_B = 0

def setup(Rpin, Gpin, Bpin):
    global pins
    global p_R, p_G, p_B, pwm_A, pwm_B
    pins = {'pin_R': Rpin, 'pin_G': Gpin, 'pin_B': Bpin}
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
    for i in pins:
        GPIO.setup(pins[i], GPIO.OUT)   # Set pins' mode is output
        GPIO.output(pins[i], GPIO.HIGH) # Set pins to high(+3.3V) to off led
    
    p_R = GPIO.PWM(pins['pin_R'], 2000)  # set Frequece to 2KHz
    p_G = GPIO.PWM(pins['pin_G'], 1999)
    p_B = GPIO.PWM(pins['pin_B'], 5000)
    
    p_R.start(100)      # Initial duty Cycle = 100(leds off)
    p_G.start(100)
    p_B.start(100)

    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)

    motorStop()
    pwm_A = GPIO.PWM(Motor_A_EN, 1000)
    pwm_B = GPIO.PWM(Motor_B_EN, 1000)

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def off():
    for i in pins:
        GPIO.output(pins[i], GPIO.HIGH)    # Turn off all leds

def setColor(col):   # For example : col = 0x112233
    R_val = (col & 0xff0000) >> 16
    G_val = (col & 0x00ff00) >> 8
    B_val = (col & 0x0000ff) >> 0

    R_val = map(R_val, 0, 255, 0, 100)
    G_val = map(G_val, 0, 255, 0, 100)
    B_val = map(B_val, 0, 255, 0, 100)
    
    p_R.ChangeDutyCycle(100-R_val)     # Change duty cycle
    p_G.ChangeDutyCycle(100-G_val)
    p_B.ChangeDutyCycle(100-B_val)

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
#    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
#    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def motorStop():
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)

def motor(status, direction, speed):
    global pwm_A, pwm_B
    if status == 0: # stop
        motorStop()
    else:
        GPIO.output(Motor_A_EN, GPIO.HIGH)
        GPIO.output(Motor_B_EN, GPIO.HIGH)
        if direction == Dir_forward:
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        if direction == Dir_backward:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)
    return direction

def loop():
    pwm.set_pwm_freq(60)
    print('Moving servo on channel 0, press Ctrl+C to quit...')
    while True:
        for col in colors:
            setColor(col)
            time.sleep(0.5)
        pwm.set_pwm(dir_ch, 0, dir_mid)
        time.sleep(1)
        pwm.set_pwm(dir_ch, 0, dir_mid+100)
        time.sleep(1)
        pwm.set_pwm(dir_ch, 0, dir_mid-70)
        time.sleep(1)
        motor(1, Dir_backward, 100)
        time.sleep(5)
        motor(0, 0, 0)    # stop
        time.sleep(5)
        motor(1, Dir_forward, 30)
        time.sleep(5)
        motor(0, 0, 0)    # stop
        time.sleep(5)

def destroy():
    p_R.stop()
    p_G.stop()
    p_B.stop()
    off()
    motorStop()
    GPIO.cleanup()

if __name__ == "__main__":
    print('Press Ctrl+C to end the program')
    try:
        setup(R, G, B)
        loop()
    except KeyboardInterrupt:
        destroy()

