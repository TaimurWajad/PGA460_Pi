import serial
import time 
import RPi.GPIO as GPIO


#pin definitions
UART_SEL0 = 22
UART_SEL1 = 17
ULTRASONIC_PWR_EN = 6     # Rev 1 and below is GPIO6, Rev 2 and up boards is GPIO21

GPIO.setmode(GPIO.BCM)
GPIO.setup(UART_SEL0, GPIO.OUT)
GPIO.setup(UART_SEL1, GPIO.OUT)
GPIO.setup(ULTRASONIC_PWR_EN, GPIO.OUT)

#SEL0=0, SEL1=0 -> SENSOR_1, SEL0=0,SEL1=1 SEMSPR_2
GPIO.output(ULTRASONIC_PWR_EN,GPIO.LOW)   #DISABLE POWER TO ULTRA SENSORS
GPIO.output(UART_SEL1,GPIO.LOW)
GPIO.output(UART_SEL0,GPIO.LOW)


# Function to send AT command and receive response
def send_message(command, timeout=1):
    ser.write((command + '\r').encode())  # Send the command
    time.sleep(timeout)
    response = ser.read_all().decode()    # Read the response
    return response



ser = serial.Serial ("/dev/ttyAMA5", 9600)    #Open port with baud rate on UART5 for MUX
count = 0
loop = 2
while count < loop:
    response = send_message('test_1')
    print('Read from UART5 Sensor 1:', response)
    time.sleep(1.5)
    count = count + 1
    

#Sensor 2 select    
GPIO.output(UART_SEL1,GPIO.LOW)
GPIO.output(UART_SEL0,GPIO.HIGH)
    
count = 0
while count < loop:
    response = send_message('test_2')
    print('Read from UART5 Sensor 2:', response)
    time.sleep(1.5)
    count = count + 1


ser.close()
