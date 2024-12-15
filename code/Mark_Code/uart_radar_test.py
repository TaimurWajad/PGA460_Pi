import serial
import time 
import RPi.GPIO as GPIO


#pin definitions
RADAR_PWR_EN = 16     # enable power to to radar

GPIO.setmode(GPIO.BCM)
GPIO.setup(RADAR_PWR_EN, GPIO.OUT)

#SEL0=0, SEL1=0 -> SENSOR_1, SEL0=0,SEL1=1 SEMSPR_2
GPIO.output(RADAR_PWR_EN,GPIO.LOW)   #DISABLE POWER TO ULTRA SENSORS


# Function to send data on UART to radar board
def send_message(command, timeout=1):
    ser.write((command + '\r').encode())  # Send the command
    time.sleep(timeout)
    response = ser.read_all().decode()    # Read the response
    return response



ser = serial.Serial ("/dev/ttyAMA4", 115200)    #Open port with baud rate on UART5 for MUX
count = 0
loop = 2
while count < loop:
    response = send_message('test_1')
    print('Read from UART4 Sensor 1:', response)
    time.sleep(1.5)
    count = count + 1
