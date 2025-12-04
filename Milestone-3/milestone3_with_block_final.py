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
    
def pause_turn():
    '''
    pauses the code less for turning
    '''
    time.sleep(1)

def wait_string():
    '''
    waits until there is a response in the serial monitor before reading it
    '''
    string = SER.readline().decode('utf-8').strip()
    while len(string) == 0:
        time.sleep(0.1)
        string = SER.readline().decode('utf-8').strip()
    return string

def sensor_readings():
    '''
    to check the sensors
    '''
    SER.write(b'getData\n')
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
    frontR = (float(data[start:end]) - 1.5) / 2.54
    
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
def move_distance(input):
    '''
    moves a certain amount of input blocks
    '''
    block = 0
    while block < input:
        SER.write(b"w 13\n")
        print("moving 1 block forward")
        pause()
        block += 1
    
def move_to_wall():
    '''
    drive commands to move in a straight line until a wall is reached
    '''
    sensor_readings()
    i = (frontL + frontR)/2
    block = round((i  - 2.5), 2)
    dist = "w " + str(block) + "\n"
    dist_encode = dist.encode("utf-8")
    print("moving forward: ", block)
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
        angle = b"r 90 \n"
        SER.write(b"r 90 \n")
        print("turning 180")
        pause()
    elif i == "b":
        angle = b"r 90 \n"
        print("turning right 90")
    else:
        angle = b"l 90 \n"
        print("turning left 90")
    SER.write(angle)
    pause_turn()

def face_north(i):
    '''
    turns the robot north for any position
    '''
    if i == "l":
        angle = b"r 90\n"
        print("turning right 90")
    elif i == "r":
        angle = b"l 90\n"
        print("turning left 90")
    elif i == "b":
        angle = b"r 90 \n"
        SER.write(b"r 90 \n")
        pause()
        print("turning 180")
    else:
        angle = b"r 0\n"
    SER.write(angle)
    pause_turn()

def face_south(i):
    '''
    turns the robot south for any position
    '''
    if i == "l":
        angle = b"l 90\n"
        print("turning left 90")
    elif i == "r":
        angle = b"r 90\n"
        print("turning right 90")
    elif i == "b":
        angle = b"r 0\n"
    else:
        angle = b"r 90\n"
        SER.write(b"r 90\n")
        print("turning 180")
        pause()
    SER.write(angle)
    pause_turn()

def face_east(i):
    '''
    turns the robot east for any position
    '''
    if i == "l":
        angle = b"r 90\n"
        SER.write(b"r 90\n")
        print("turning 180")
        pause()
    elif i == "r":
        angle = b"r 0\n"
    elif i == "b":
        angle = b"l 90\n"
        print("turning left 90")
    else:
        angle = b"r 90\n"
        print("turning right 90")
    SER.write(angle)
    pause_turn()

def D_move():
    '''
    moves to the loading zone from any of the D spots
    '''
    pause()
    move_to_wall()
    pause()
    face_north("l")
    pause()
    move_distance(1)
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
    pause()
    move_distance(1)
    pause()
    
def center():
    '''
    centers the block in the 1ft x 1ft square
    '''
    sensor_readings()
    time.sleep(1)
    fR = round((frontR - 2)) % 12
    fR_move = round((frontR - 2) % 12, 2)
    dist = "w " + str(fR_move) + "\n"
    dist_encode = dist.encode("utf-8")
    fL = round((frontL - 2)) % 12
    if (fR or fL) > 1.5:
        print("not centered in the path front \n remainder:", fR)
        print("moving forward: ", fR_move)
        SER.write(dist_encode)
        pause()
    sensor_readings()
    r = round((right - 2.5)) % 12
    r_move = round((right - 2.5) % 12, 2)
    dist = "w " + str(r_move) + "\n"
    dist_encode = dist.encode("utf-8")
    l = round((left - 2.5)) % 12
    if (l or r) > 1.5:
        print("not centered in the path L/R \nremainder:", l, r)
        face_north("l")
        print("moving forward: ", r_move)
        SER.write(dist_encode)
        pause()
        face_north("r")

def orient():
    '''
    orients to face the wall
    '''
    orientation = True

    #ORIENT TO FACE THE WALL:
    while orientation:
        sensor_readings()
        time.sleep(LOOP_PAUSE_TIME)
        if frontR > 30:
            print("wall is far away")
            face_north("l")
            print("turning 90 \n", "frontR: ", frontR, "frontL: ", frontL)
        elif abs(frontR-frontL) > 0.3:
            SER.write(b"r 7\n") #need to figure out what this range of rotation should be
            print("great than 1 \n", "frontR: ", frontR, "frontL: ", frontL)
        elif (frontR-frontL) > 0.005:
            SER.write(b"l 1\n") #need to figure out what this range of rotation should be
            print("front right larger\n", "frontR: ", frontR, "frontL: ", frontL)
        elif (frontL-frontR) > 0.005:
            SER.write(b"r 1\n") #need to figure out what this range of rotation should be
            print("front left larger\n", "frontR: ", frontR, "frontL: ", frontL)
        else:
            orientation = False
            print("Wall Oriented!")
            print("front left:", frontL, "front right:", frontR)
            print("left:", left, "right", right)
        pause_turn()

#Localization functions
def wall_dis():
    '''
    calculates how far away from the wall the bot is and converts into number of tiles away
    '''
    measurement = (frontL + frontR)/2
    print("average distance is: ", measurement)
    if 0 <= measurement <= 6:
        wall = 0
    elif 10 <= measurement <= 18:
        wall = 1
    elif 22 <= measurement <= 30:
        wall = 2
    elif 34 <= measurement <= 42:
        wall = 3
    elif 46 <= measurement <= 54:
        wall = 4
    elif 58 <= measurement <= 66:
        wall = 5
    else:
        face_north("r")
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
    print("ir sensor: ", i, "\n data: ", ir)
    if i == 1: # <- double check that 1 is black from sensor
        out = 0
    else:
        out = 1
    print("ir sensor returning: ", out)
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
        if i == 2:
            SER.write(b"r 5\n")
            pause_turn()
        time.sleep(LOOP_PAUSE_TIME)
        i += 1
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
        localize()
    return loc, pattern

def find_block(input):
    '''
    compares the sensor readings to detect the block
    '''
    block = True
    while block:
        sensor_readings()
        front = round((frontL + frontR) / 2, 2) + 1
        block_dis_round = round(block_dis, 2)
        print("front: ", front, "\n", "block: ", block_dis)
        time.sleep(2)
        if (front - block_dis_round) > 3.5:
            print("found the block")
            block_move = (block_dis_round - 8)
            if block_move > 0:
                dist = "w " + str(block_move) + "\n"
                dist_encode = dist.encode("utf-8")
                print("moving: ", block_move)
                SER.write(dist_encode)
            pause()
            sensor_readings()
            if block_dis > 4:
                SER.write(b"r 1\n")
                find_block(input)
            block = False
        elif input == "C":
            SER.write(b"r 3\n")
            pause_turn()
        else:
            SER.write(b"l 3\n")
            pause_turn()

#CODE STARTS HERE:

#initialization
B = input("Enter final location:")
sensor_readings()
SER.write(b's 0\n')
time.sleep(LOOP_PAUSE_TIME)
SER.write(b'not localized\n')
time.sleep(LOOP_PAUSE_TIME)
print("starting...")
orient()
center()
time.sleep(LOOP_PAUSE_TIME)

#localization
loc, pattern = localize()
print(loc)
print(pattern)
spot = pos_dictionary.name(loc, pattern)
print(spot)
time.sleep(5)
if spot is None:
    time.sleep(LOOP_PAUSE_TIME)
    spot = input("Enter approximate location:")
    time.sleep(5)
SER.write(b'localized\n')
time.sleep(LOOP_PAUSE_TIME)

face = ""
#movement to loading zone
if (spot[:2] == "D1"):
    face_north(spot[3])
    move_distance(1)
    face = "C"
elif (spot[:2] == "C1"):
    face_north(spot[3])
    face = "C"
elif (spot[:2] == "A8" or spot[:2] == "A6"):
    face_south(spot[3])
    move_distance(1)
    face_west("b")
    B_move()
    face = "A"
elif (spot[0] == "B" and spot[1] != "2" and spot[1] != "1"):
    face_west(spot[3])
    B_move()
    face = "A"
elif spot[1] == "8":
    face_north(spot[3])
    move_distance(2)
    face_west("f")
    B_move()
    face = "A"
elif spot[0] == "D":
    face_west(spot[3])
    sensor_readings()
    D_move()
    face = "C"
elif spot[0] == "C":
    face_south(spot[3])
    move_to_wall()
    face_west("b")
    sensor_readings()
    D_move()
    face = "C"
elif (spot[:2] == "A3"):
    face_west(spot[3])
    face = "A"
elif (spot[:2] == 'A4'):
    face_west(spot[3])
    move_distance(1)
    face = "A"
else:
    print("already in the loading zone")
    SER.write(b'light\n')
    time.sleep(2)
    SER.write(b'light\n')

time.sleep(5)
SER.write(b'w 6\n')
pause()
SER.write(b's 90\n')
time.sleep(LOOP_PAUSE_TIME)
print("starting to search for the block")
find_block(face)
time.sleep(5)
SER.write(b's 0\n')
time.sleep(5)
orient()
if face == 'A':
    move_to_wall()
    face_north('l')
    move_to_wall()
else:
    face_west('r')
    move_to_wall()
    face_north('l')
    move_to_wall()
    
#should be facing north at A1
print("Reached the loading zone!")
SER.write(b'light\n')
time.sleep(2)
SER.write(b'light\n')
time.sleep(2)
spot = "A1 forward"

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
SER.write(b's 0\n')