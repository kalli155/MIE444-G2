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

#function to wait until done before moving
def pause():
    '''
    stops the code from running if the bot is still moving
    '''
    time.sleep(5)

def wait_string():
    '''
    waits until there is a response in the serial monitor before reading it
    '''
    string = SER.readline().decode('utf-8').strip()
    print(string)
    while len(string) == 0:
        time.sleep(0.1)
        string = SER.readline().decode('utf-8').strip()
    return string

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
    global block_dis
    global ir
    
    #u1
    start = 4
    end = data.index("u2: ")
    frontL = float(data[start:end]) / 2.54
    
    #u2
    start = end + 4
    end = data.index("u3: ")
    left = float(data[start:end]) / 2.54
    
    #u3
    start = end + 4
    end = data.index("u4: ")
    frontR = float(data[start:end]) / 2.54
    
    #u4
    start = end + 4
    end = data.index("u0: ")
    right = float(data[start:end]) / 2.54
    
    #u0
    start = end + 4
    end = data.index("i0: ")
    block_dis = float(data[start:end]) / 2.54
    
    #ir
    start = end + 4
    end = start + 1
    ir = int(data[start:end])
    

#drive functions
def move_distance(i):
    '''
    moves a certain amount of input blocks
    '''
    block = i*12
    dist = "w " + str(block) + "\n"
    dist_encode = dist.encode("utf-8")
    SER.write(dist_encode)
    pause()
    
def move_to_wall():
    '''
    drive commands to move in a straight line until a wall is reached
    '''
    sensor_readings()
    i = (frontL + frontR)/2
    block = round((i  - 2.5), 2)
    dist = b"w " + str(block) + "\n"
    dist_encode = dist.encode("utf-8")
    SER.write(dist_encode)
    pause()
    sensor_readings()
    if (frontL or frontR) > 3:
        move_to_wall()

def face_west(i):
    '''
    turns the robot west for any position
    '''
    if i == "l":
        angle = b"r 0 \n"
    elif i == "r":
        angle = b"r 180 \n"
    elif i == "b":
        angle = b"r 90 \n"
    else:
        angle = b"l 90 \n"
    SER.write(angle)
    pause()

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

def face_south(i):
    '''
    turns the robot south for any position
    '''
    if i == "l":
        angle = b"l 90\n"
    elif i == "r":
        angle = b"r 90\n"
    elif i == "b":
        angle = b"r 0\n"
    else:
        angle = b"r 180\n"
    SER.write(angle)
    pause()

def face_east(i):
    '''
    turns the robot east for any position
    '''
    if i == "l":
        angle = b"r 180\n"
    elif i == "r":
        angle = b"r 0\n"
    elif i == "b":
        angle = b"l 90\n"
    else:
        angle = b"r 90\n"
    SER.write(angle)
    pause()

def D_move():
    '''
    moves to the loading zone from any of the D spots
    '''
    pause()
    move_to_wall()
    pause()
    face_north("l")
    pause()
    move_to_wall()
    pause()

def A_move():
    '''
    moves to the loading zone from any of the A spots
    '''
    pause()
    move_to_wall()
    pause()

def B_move():
    '''
    moves to the loading zone from any of the B spots
    '''
    pause()
    move_to_wall()
    pause()
    face_north("l")
    pause()
    move_to_wall()
    pause()
    face_west("f")
    A_move()
    pause()
    
def center():
    '''
    centers the block in the 1ft x 1ft square
    '''
    sensor_readings()
    pause()
    fR = round((frontR - 2.5)) % 12
    fR_move = round((frontR - 2.5) % 12, 2) / 12
    fL = round((frontL - 2.5)) % 12
    if (fR or fL) > 1:
        print("not centered in the path front \n remainder:", fR)
        move_distance(fR_move) # it will move forward a lot... maybe want to keep it a function of the remainder??
        pause()
    sensor_readings()
    r = round((right - 2.5)) % 12
    r_move = round((right - 2.5) % 12, 2) / 12
    l = round((left - 2.5)) % 12
    if (l or r) > 1:
        print("not centered in the path L/R \n remainder:", l, r)
        face_north("l")
        pause()
        move_distance(r_move)
        pause()
        face_north("r")
        pause()

#Localization functions
def wall_dis():
    '''
    calculates how far away from the wall the bot is and converts into number of tiles away
    '''
    measurement = (frontL + frontR)/2
    print("average distance is: ", measurement)
    if 0 <= measurement <= 4:
        wall = 0
    elif 12 <= measurement <= 16:
        wall = 1
    elif 24 <= measurement <= 28:
        wall = 2
    elif 36 <= measurement <= 40:
        wall = 3
    elif 48 <= measurement <= 52:
        wall = 4
    elif 60 <= measurement <= 64:
        wall = 5
    else:
        face_north("r")
        pause()
        sensor_readings()
        print("right:", right)
        if 0 < right < 8:
            wall = 0
        elif 8 < right < 20:
            wall = 1
        elif 20 < right < 32:
            wall = 2
        elif 32 < right < 44:
            wall = 3
        elif 44 < right < 56:
            wall = 4
        elif 56 < right < 68:
            wall = 5
        else:
            print("Error")
            wall = -1
        time.sleep(LOOP_PAUSE_TIME)
        face_north("l")
        pause()
    return wall

def pattern_col(i):
    '''
    converts the black or white to 0 or 1
    '''
    if i == '1':  # <- double check that 1 is black from sensor
        out = 0
    else:
        out = 1
    return out

def localize():
    '''
    generates the location and pattern lists needed to localize
    '''
    i = 0
    loc = [0, 0, 0, 0]
    pattern = [0, 0, 0, 0]
    while i < 4:
        sensor_readings()
        pattern[i] = pattern_col(ir)
        loc[i] = wall_dis()
        time.sleep(LOOP_PAUSE_TIME)
        face_north("l")
        pause()
        i += 1
    return loc, pattern

def find_block():
    '''
    compares the sensor readings to detect the block
    '''
    sensor_readings()
    front = (frontL - frontR) / 2 #add in a way to account for the difference in placement between the front sensors and the block sensor
    if (front - block_dis) > 1:
        print("found the block")
        #center relative to the block/might already be because of the sensor placement
        #move until close to the block
        
def grab_block(angle):
    '''
    sends the commands to rotate the servo to 
    '''
    ang = b"s " + str(angle) + "\n"
    ang_encode = ang.encode("utf-8")
    SER.write(ang_encode)

def main():
    '''
    main function to run (only is a function so that I can run simultaneously)
    '''
    orientation = True

    #ORIENT TO FACE THE WALL:
    while orientation:
        time.sleep(LOOP_PAUSE_TIME)
        sensor_readings()
        if frontR > 60:
            face_north("l")
            print("turning 90 \n", "frontR: ", frontR, "frontL: ", frontL)
        elif abs(frontR-frontL) > 0.25:
            SER.write(b"r 7\n") #need to figure out what this range of rotation should be
            print("great than 1 \n", "frontR: ", frontR, "frontL: ", frontL)
        elif (frontR-frontL) > 0.05:
            SER.write(b"l 1\n") #need to figure out what this range of rotation should be
            print("front right larger\n", "frontR: ", frontR, "frontL: ", frontL)
        elif (frontL-frontR) > 0.05:
            SER.write(b"r 1\n") #need to figure out what this range of rotation should be
            print("front left larger\n", "frontR: ", frontR, "frontL: ", frontL)
        else:
            orientation = False
            print("Wall Oriented!")
            print("front left:", frontL, "front right:", frontR)
            print("left:", left, "right", right)

    #LOCALIZATION:
    
    center()
    time.sleep(LOOP_PAUSE_TIME)
    loc, pattern = localize()

    #check for one of 2 locations that would return the same thing
    if pattern == [0, 0, 0, 0]:
        sensor_readings()
        print("can't localize here")
        if frontL > 12:
            move_distance(1)
            pause()
        elif left > right:
            face_north("r")
            pause()
            move_distance(1)
            pause()
        elif right > left:
            face_north("l")
            pause()
            move_distance(1)
            pause()
        else:
            face_north("b")
            pause()
            move_distance(1)
            pause()
        time.sleep(LOOP_PAUSE_TIME)
        sensor_readings()
        pause()
        loc, pattern = localize()

    print(loc)
    print(pattern)
    spot = pos_dictionary.name(loc, pattern)
    print(spot)
    time.sleep(LOOP_PAUSE_TIME)

    #movement to loading zone
    if (spot[1] == "1" and spot[0] != "B" and spot[0] != "A"):
        face_north(spot[3])
        move_to_wall()
    elif (spot[:2] == "A8" or spot[:2] == "A6"):
        face_south(spot[3])
        move_distance(1)
        face_west("b")
        B_move()
    elif (spot[0] == "B" and spot[1] != "2" and spot[1] != "1"):
        face_west(spot[3])
        B_move()
    elif spot[1] == "8":
        face_north(spot[3])
        move_distance(2)
        face_west("f")
        B_move()
    elif spot[0] == "D":
        face_west(spot[3])
        sensor_readings()
        D_move()
    elif spot[0] == "C":
        face_south(spot[3])
        move_to_wall()
        face_west("b")
        sensor_readings()
        D_move()
    elif (spot[0] == "A" and spot[1] != "2"):
        face_west(spot[3])
        A_move()
    else:
        print("Reached the loading zone! *not A1")
        SER.write(b'light')
        time.sleep(2)
        SER.write(b'light')

    #confirm at loading zone
    time.sleep(LOOP_PAUSE_TIME)
    sensor_readings()
    loc, pattern = localize()
    print(loc)
    print(pattern)
    spot = pos_dictionary.name(loc, pattern)
    print(spot)
    if spot[:2] == "A1":
        print("Reached the loading zone!")
        SER.write(b'light')
        time.sleep(2)
        SER.write(b'light')

    time.sleep(5)
    find_block()
    time.sleep(5)
    grab_block(90)

    #head to the final destination
    if spot[:2] != "A1":
        face_north(spot[3])
        move_to_wall()
        face_west("f")
        move_to_wall()
        spot = "A1 l"

    if B[1] == "1":
        face_south(spot[3])
        move_to_wall()
        face_east("b")
        move_distance(2)
        face_north("r")
        move_to_wall()
    else:
        face_east(spot[3])
        move_to_wall()
        face_south("r")
        move_to_wall()
        face_east("b")
        if B[1] == "2":
            move_distance(2)
            face_north("r")
            move_to_wall()
        else:
            move_to_wall()
            if B[1] == "3":
                face_north("r")
                move_to_wall()
            else:
                face_south("r")
                move_to_wall()
    print("Reached the final destination!")
    grab_block(0)

#user input for the final drop off zone
B = input("Enter final location:")
sensor_readings()
main()
