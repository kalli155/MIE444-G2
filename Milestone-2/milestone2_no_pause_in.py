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

#sensor reading function
def extractNum(input):
    start = 0
    end = input.index('\r')
    return input[start:end]

#function to wait until done before moving
def pause():
    '''
    stops the code from running if the bot is still moving
    '''
    time.sleep(5)

def wait_string():
    string = SER.readline().decode('utf-8').strip()
    while len(string) == 0:
        time.sleep(0.1)
        string = SER.readline().decode('utf-8').strip()
    return string

def sensor_readings():
    '''
    to check the sensors
    '''
    SER.write(b'u1\n')
    u1 = wait_string()
    global frontL
    frontL = float(u1) / 2.54
    
    SER.write(b'u2\n')
    u2 = wait_string()
    global left
    left = float(u2) / 2.54

    SER.write(b'u3\n')
    u3 = wait_string()
    global frontR
    frontR = float(u3) / 2.54

    SER.write(b'u4\n')
    u4 = wait_string()
    global right
    right = float(u4) / 2.54

    SER.write(b'i0\n')
    i0 = wait_string()
    global ir
    ir = float(i0)
    print("ir :", ir)


#check to make sure the walls aren't too close
#def check_walls(): #update new values for our margins <- how to continuously run this
#    '''
#    function checks to see if the bot is too close to the walls and stops motion if so
#    '''
#    sensor_readings()
#    global running
#    running = True
#    while running:
#        if frontL or frontR < 2:
#            SER.write(b"x\n")
#        elif right or left < 1:
#            SER.write(b"x\n")

#drive functions
def move_straight(i):
    '''
    drive commands to move in a straight line until a wall is reached
    '''
    block = 12*(int(i)-1)
    dist = b"w " + str(block) + "\n"
    SER.write(dist)
    pause()

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

def D_move(i):
    '''
    moves to the loading zone from any of the D spots
    '''
    move_straight(i)
    pause()
    face_north("l")
    pause()
    move_straight("4")
    pause()

def A_move(i):
    '''
    moves to the loading zone from any of the A spots
    '''
    move_straight(i)
    pause()

def B_move(i):
    '''
    moves to the loading zone from any of the B spots
    '''
    i = int(i) - 3
    move_straight(i)
    pause()
    face_north("l")
    pause()
    move_straight("2")
    pause()
    face_west("f")
    pause()
    A_move("4")

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
    if i == 'True':
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
        print("i: ", i)
        sensor_readings()
        pattern[i] = pattern_col(ir)
        loc[i] = wall_dis()
        time.sleep(LOOP_PAUSE_TIME)
        face_north("l")
        pause()
        i += 1
    return loc, pattern

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

    #LOCALIZATION:
    time.sleep(LOOP_PAUSE_TIME)
    loc, pattern = localize()

    #check for one of 2 locations that would return the same thing
    if pattern == [0, 0, 0, 0]:
        sensor_readings()
        print("can't localize here")
        if frontL > 12:
            move_straight("2")
            pause()
        elif left > right:
            face_north("r")
            pause()
            move_straight("2")
            pause()
        elif right > left:
            face_north("l")
            pause()
            move_straight("2")
            pause()
        else:
            face_north("b")
            pause()
            sensor_readings()
            pause()
            move_straight("2")
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
        if spot[0] == "D":
            move_straight("4")
        else:
            move_straight("3")
    elif (spot[:2] == "A8" or spot[:2] == "A6"):
        face_south(spot[3])
        move_straight("2")
        face_west("b")
        B_move(spot[1])
    elif (spot[0] == "B" and spot[1] != "2" and spot[1] != "1"):
        face_west(spot[3])
        B_move(spot[1])
    elif spot[1] == "8":
        face_north(spot[3])
        move_straight("3")
        face_west("f")
        B_move(spot[1])
    elif spot[0] == "D":
        face_west(spot[3])
        D_move(spot[1])
    elif spot[0] == "C":
        face_south(spot[3])
        move_straight("2")
        face_west("b")
        D_move(spot[1])
    elif (spot[0] == "A" and spot[1] != "2"):
        face_west(spot[3])
        A_move(spot[1])
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

    #head to the final destination
    if spot[:2] != "A1":
        if spot[0] == "B":
            face_north(spot[3])
            move_straight("2")
        face_west("f")
        move_straight(spot[1])
        spot = "A1 l"
        print(spot)

    if B[1] == "1":
        face_south(spot[3])
        move_straight("4")
        face_east("b")
        move_straight("3")
        face_north("r")
        move_straight("2")
    else:
        face_east(spot[3])
        move_straight("4")
        face_south("r")
        move_straight("2")
        face_east("b")
        if B[1] == "2":
            move_straight("3")
            face_north("r")
            move_straight("2")
        else:
            move_straight("5")
            if B[1] == "3":
                face_north("r")
                move_straight("2")
            else:
                face_south("r")
                move_straight("3")

    running = False
    print("Reached the final destination!")

## CODE STARTS HERE FOR MOVING

#user input for the final drop off zone
B = input("Enter final location:")
sensor_readings()
main()

#if __name__ == '__main__':
#    Thread(target = main).start()
#    Thread(target = check_walls).start()