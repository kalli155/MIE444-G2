import time
from threading import Thread
import serial
from datetime import datetime
import pos_dictionary

############## Constant Definitions Begin ##############
BAUDRATE = 9600 # Baudrate in bps
PORT_SERIAL = 'COM6' # COM port identification
TIMEOUT_SERIAL = 0.1 # Serial port timeout, in seconds
LOOP_PAUSE_TIME = 1 #seconds
COMMAND_PAUSE_TIME = 0.1 #seconds #can change this based on how quick the response time is

SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)

def wait_string():
    string = SER.readline().decode('utf-8').strip()
    print(string)
    while len(string) == 0:
        time.sleep(0.1)
        string = SER.readline().decode('utf-8').strip()
    return string

def pause():
    '''
    stops the code from running if the bot is still moving
    '''
    time.sleep(5)

def sensor_readings():
    '''
    to check the sensors
    '''
    SER.write(b'getData')
    data = wait_string()
    
    global frontL
    global frontR
    global left
    global right
    global ir
    
    #u1
    start = 4
    end = data.index("u2: ")
    frontL = float(data[start:end])
    
    #u2
    start = end + 4
    end = data.index("u3: ")
    left = float(data[start:end])
    
    #u3
    start = end + 4
    end = data.index("u4: ")
    frontR = float(data[start:end])
    
    #u4
    start = end + 4
    end = data.index("i0: ")
    right = float(data[start:end])
    
    #ir
    start = end + 4
    end = start + 1
    ir = int(data[start:end])

def face_north(i):
    '''
    turns the robot north for any position
    '''
    if i == "l":
        angle = b"r 90\n"
    elif i == "r":
        angle = b"l 90\n"
    elif i == "b":
        angle = b"r 180\n"
    else:
        angle = b"r 0\n"
    SER.write(angle)
    pause()

orientation = True

#ORIENT TO FACE THE WALL:
while orientation:
    time.sleep(LOOP_PAUSE_TIME)
    sensor_readings()
    if frontR > 60:
        face_north("l")
        print("turning 90 \n", "frontR: ", frontR, "frontL: ", frontL)
    elif abs(frontR-frontL) > 0.5:
        SER.write(b"r 7\n") #need to figure out what this range of rotation should be
        print("great than 1 \n", "frontR: ", frontR, "frontL: ", frontL)
    elif (frontR-frontL) > 0.1:
        SER.write(b"l 1\n") #need to figure out what this range of rotation should be
        print("front right larger\n", "frontR: ", frontR, "frontL: ", frontL)
    elif (frontL-frontR) > 0.1:
        SER.write(b"r 1\n") #need to figure out what this range of rotation should be
        print("front left larger\n", "frontR: ", frontR, "frontL: ", frontL)
    else:
        orientation = False
        print("Wall Oriented!")
        print("front left:", frontL, "front right:", frontR)
        print("left:", left, "right", right)