import RPi.GPIO as gpio
import time

def motor_control(mode,dc1,dc2,sec):
    if mode == 1:
        gpio.output(in1, True)
        gpio.output(in2, False)
        gpio.output(in3, False) 
        gpio.output(in4, True)
    if mode == 0:
        gpio.output(in1, False)
        gpio.output(in2, True)
        gpio.output(in3, True) 
        gpio.output(in4, False)
    p.ChangeDutyCycle(dc1)
    q.ChangeDutyCycle(dc2)
    time.sleep(sec)
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(0)

ena = 2
enb = 3
in1 = 17 #motor 1
in2 = 22 #motor 1
in3 = 23 #motor 2
in4 = 24 #motor 2

gpio.setmode(gpio.BCM)
gpio.setup(in1, gpio.OUT)
gpio.setup(in2, gpio.OUT)
gpio.setup(in3, gpio.OUT)
gpio.setup(in4, gpio.OUT)
gpio.setup(ena, gpio.OUT)
gpio.setup(enb, gpio.OUT)
p = gpio.PWM(ena,50)
q = gpio.PWM(enb,50)
p.start(0)
q.start(0)
while True:
    try:
        mode = input("set direction, 1 for forward, 0 for reverse:")
        dc1 = input("set speed for motor 1:")
        dc2 = input("set speed for motor 2:")
        sec = input("set motor run time:")
        motor_control(mode,dc1,dc2,sec)
    except KeyboardInterrupt:
        print "cleanup gpio"
        gpio.cleanup()
    except EOFError:
        print "quiting"
        gpio.cleanup()
        exit()
