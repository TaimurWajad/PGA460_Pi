import RPi.GPIO as GPIO
import time

#pin definitions
LARA_PWR = 2
LARA_RES = 3
UART_SEL1 = 17
RDR_SYNC = 27
UART_SEL0 = 22
SHUTDOWN = 10
RUN = 11
FAN_PWM = 18
RADAR_PWR_EN = 16

DUTY_CYCLE = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(LARA_PWR, GPIO.OUT)
GPIO.setup(LARA_RES, GPIO.OUT)
GPIO.setup(UART_SEL1, GPIO.OUT)
GPIO.setup(RDR_SYNC, GPIO.OUT)
GPIO.setup(UART_SEL0, GPIO.OUT)
GPIO.setup(SHUTDOWN, GPIO.IN)
GPIO.setup(RUN, GPIO.OUT)
GPIO.setup(FAN_PWM, GPIO.OUT)
GPIO.setup(RADAR_PWR_EN, GPIO.OUT)

pwm = GPIO.PWM(FAN_PWM, 50)  #initialize the pwm at 100hz
pwm1 = GPIO.PWM(RDR_SYNC, 50)  #initialize the pwm at 100hz

pwm.start(DUTY_CYCLE)
pwm1.start(DUTY_CYCLE)

print("starting")

try:
	while 1:
		GPIO.output(LARA_PWR,GPIO.HIGH)
		GPIO.output(LARA_RES,GPIO.HIGH)
		GPIO.output(UART_SEL1,GPIO.HIGH)
		GPIO.output(UART_SEL0,GPIO.HIGH)
		GPIO.output(RUN,GPIO.HIGH)
		GPIO.output(RADAR_PWR_EN,GPIO.HIGH)
		
		
		time.sleep(1)

		GPIO.output(LARA_PWR,GPIO.LOW)
		GPIO.output(LARA_RES,GPIO.LOW)
		GPIO.output(UART_SEL1,GPIO.LOW)
		GPIO.output(UART_SEL0,GPIO.LOW)
		GPIO.output(RUN,GPIO.LOW)
		GPIO.output(RADAR_PWR_EN,GPIO.LOW)

		if GPIO.input(SHUTDOWN):
			print("SHUTDOWN is HIGH")

		time.sleep(1)
		


except KeyboardInterrupt:
	print("keyboard interrupt")

finally:
	print("cleanup")
	pwm.STOP()
	GPIO.cleanup()


		
