#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, Image, ImageFile, Font
from pybricks.messaging import BluetoothMailboxServer, BluetoothMailboxClient, LogicMailbox, NumericMailbox, TextMailbox
from threading import Thread
from random import choice
from math import fmod
import sys
import os
import math
import struct


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
# Any (parts) of program taken from Anton's Mindstorms Hacks are used under the;
# MIT License: Copyright (c) 2021 Anton's Mindstorms
# MIT License: Copyright (c) 2023 Mr Jos for the rest of the code

#####################################################################
#####################################################################
##########~~~~~PROGRAM WRITTEN BY JOZUA VAN RAVENHORST~~~~~##########
##########~~~~~~~~~~~~~ROBOTMAK3RS REMIX 42144~~~~~~~~~~~~~##########
##########~~~~~~~~~~~~~YOUTUBE CHANNEL: MR JOS~~~~~~~~~~~~~##########
#####################################################################
##########~~~~~~~~~~~~~EV3 ADVANCED MACHINERY~~~~~~~~~~~~~~##########
#####################################################################
#####################################################################


##########~~~~~~~~~~HARDWARE CONFIGURATION~~~~~~~~~~##########
ev3 = EV3Brick()                                                                    #Name we will be using to make the brick do tasks

#   Motors definition
turret_rot  = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)          #Turret motor,    +direction = turning clockwise
main_boom   = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)          #Main boom motor, +dir = boom up
sub_boom    = Motor(Port.C)                                                         #Sub boom motor,  +dir = boom up
claw        = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)          #4-Claw motor,    +dir = claw open
#   Sensor definition
touch_turret_cw  = TouchSensor(Port.S2)
touch_turret_ccw = TouchSensor(Port.S3)


##########~~~~~~~~~~HOMING POSITION ANGLES WHEN SENSOR ACTIVATED~~~~~~~~~~##########
valve_center_to_end     = 420


##########~~~~~~~~~~GEARING~~~~~~~~~~##########


##########~~~~~~~~~~MAXIMUM SPEED, MAXIMUM ACCELERATION, MAXIMUM POWER~~~~~~~~~~##########
turret_rot.control.limits(1200,  200, 100)                                          #Max allowed speed(°/sec), set the accel-/deceleration(°/s²) and max power(%)
main_boom.control.limits( 1200, 2400, 100)
sub_boom.control.limits(  1200, 2400, 100)
claw.control.limits(      1200, 2400, 100)
nominal_speed_boom_up   = 1200                                                      #Normal speed for moving an arm up
nominal_speed_boom_down = 400                                                       #Normal speed for moving the arm down
claw_speed              = 1200                                                      #Normal speed for opening/closing the 4-claw
turret_speed            = 300


##########~~~~~~~~~~MAXIMUM ACCELERATION AND MAXIMUM ANGLE TO SAY A MOVEMENT IS FINISHED~~~~~~~~~~##########
#claw.control.target_tolerances(1000, 10)                                           #Allowed deviation from the target before motion is considered complete. (deg/s, deg) (1000, 10)


##########~~~~~~~~~~BLUETOOTH SETUP, SERVER SIDE~~~~~~~~~~##########                #This is not used in this project I use my standard template to program all my projects
#server = BluetoothMailboxServer()
#commands_bt_text = TextMailbox('commands text', server)                            #Main mailbox for sending commands and receiving feedback to/from other brick
#yaw_base_bt_zeroing = NumericMailbox('zero position yaw', server)                  #Mailbox for sending theta1 homing position


##########~~~~~~~~~~CREATING AND STARTING A TIMER~~~~~~~~~~##########               #This is not used in this project I use my standard template to program all my projects
#timer_calibrate  = StopWatch()                                                     #Creating the timer that will be used for calibration


##########~~~~~~~~~~BUILDING GLOBAL VARIABLES~~~~~~~~~~##########
main_boom_down_to_up    = 50                                                        #Amount of 360° rotations needed to go from full down to up
main_boom_up_to_down    =  8                                                        #Rotations for the main arm to go completely down
sub_boom_down_to_up     = 36                                                        #Rotations for the end arm to go completely up
sub_boom_up_to_down     = 15                                                        #Rotations for the end arm to go completely down
claw_open_to_closed     =  6                                                        #Rotations to close the 4-claw
claw_closed_to_open     =  6                                                        #Rotations to open the 4-claw
claw_full_grip          =  5                                                        #Amount of normal closing operations to have a strong grip (2bar)
grip_level              =  0                                                        #Current grip level for the 4-claw, 0=no power


##########~~~~~~~~~~BRICK STARTUP SETTINGS~~~~~~~~~~##########
ev3.speaker.set_volume(volume=80, which='_all_')                                    #Set the volume for all sounds (speaking and beeps etc)
ev3.speaker.set_speech_options(language='en', voice='m7', speed=None, pitch=None)   #Select speaking language, and a voice (male/female)
small_font  = Font(size=6)                                                          # 6 pixel height for text on screen
normal_font = Font(size=10)                                                         #10 pixel height for text on screen
big_font    = Font(size=16)                                                         #16 pixel height for text on screen
ev3.screen.set_font(normal_font)                                                    #Choose a preset font for writing next texts
ev3.screen.clear()                                                                  #Make the screen empty (all pixels white)
#ev3.speaker.beep()                                                                 #Brick will make a beep sound 1 time
ev3.light.off()                                                                     #Turn the lights off on the brick


##########~~~~~~~~~~CREATING A FILE THAT IS SAVED OFFLINE~~~~~~~~~~##########       #This is not used in this project I use my standard template to program all my projects
#os.remove("calibrationdata.txt")                                                   #This is for removing the file we will make next, this is for debugging for me, keep the # in front of it
#create_file = open("calibrationdata.txt", "a")                                     #Create a file if it does not exist and open it, if it does exist, just open it
#create_file.write("")                                                              #Write the default values to the file, for first ever starttup so it holds values
#create_file.close()                                                                #Close the file again, to be able to call it later again

#with open("calibrationdata.txt") as retrieve_data:                                 #Open the offline data file
#    data_retrieval_string = retrieve_data.read().splitlines()                      #The data is in the Type: String , read the complete file line by line
#if len(data_retrieval_string) < 12: data_background_offline = limits_scanned       #Check if there are 12 values in the string list, if not then it is first start of this program ever
#else:                                                                              #If there are 12 then it will convert the String to a Integer list.
#    data_background_offline = []
#    for x in data_retrieval_string:
#        data_background_offline.append(int(x))
#limits_scanned = data_background_offline                                           #The background color is now defined from the offline file (last calibration done)


##########~~~~~~~~~~CREATING FUNCTIONS THAT CAN BE CALLED TO PERFORM REPETITIVE OR SIMULTANEOUS TASKS~~~~~~~~~~##########
#def save_offline_data():                                                           #This definition will save the current background limits to the offline file, if it is called
#    with open("calibrationdata.txt", "w") as backup_data:
#        for current_data in limits_scanned:
#            backup_data.write(str(current_data) + "\n")
   

def clear_screen():
    ev3.screen.clear()                                                              #Empty the complete screen on the EV3 brick
    ev3.screen.draw_text(103, 114, "Mr Jos creation", text_color=Color.BLACK, background_color=Color.WHITE)     #Write text on the EV3 screen on the XY grid


##########~~~~~~~~~~Scaling block for changing 0-255 PS4-remote input to range of choice~~~~~~~~~~########## CREDITS TO: ANTON'S MINDSTORMS
def scale(val, src, dst):                                                           #Definition for scaling the PS4 analog output to an useable value
    return (float(val-src[0]) / (src[1]-src[0])) * (dst[1]-dst[0])+dst[0]


##########~~~~~~~~~~CREATING MULTITHREADS~~~~~~~~~~##########                       #This is not used in this project I use my standard template to program all my projects
#sub_white_scanner = Thread(target=check_color_white)                               #Creating a multithread so the definition can run at the same time as the main program, if it's called


##########~~~~~~~~~~MAIN PROGRAM~~~~~~~~~~##########
clear_screen()


            ##########~~~~~~~~~~PS4 CONTROL~~~~~~~~~~##########
##########################################################################
##########~~~~~~~~~~THANKS TO ANTON'S MINDSTORMS HACKS~~~~~~~~~~##########
##########~~~~~~~~~~  https://antonsmindstorms.com/   ~~~~~~~~~~##########
##########################################################################
infile_path = "/dev/input/event4"
in_file = open(infile_path, "rb")

FORMAT = 'llHHI'    
EVENT_SIZE = struct.calcsize(FORMAT)
event = in_file.read(EVENT_SIZE)

while event:
    (tv_sec, tv_usec, ev_type, code, value) = struct.unpack(FORMAT, event)
    
    #type1 push button 0/1:
    # 544 arrow up       17
    # 546 arrow left     16
    # 545 arrow down     17
    # 547 arrow right    16
    # 314 select
    # 316 middle button
    # 315 start button
    # 307 triangle
    # 308 square
    # 304 cross
    # 305 circle
    # 317 left joystick pushdown
    # 318 right joystick pushdown
    # 310 L1
    # 311 R1
    # 312 L2
    # 313 R2

    #type3 analog 0-255:
    # 0 left joystick X-axis
    # 1 left joystick Y-axis
    # 2 L2 paddle
    # 3 right joystick X-axis
    # 4 right joystick Y-axis
    # 5 R2 paddle

    if ev_type == 1: # A button was pressed or released.
        if code == 304 and value == 1 and grip_level < claw_full_grip:              #Close the grabber
            claw.run_time(-claw_speed, 4000, then=Stop.COAST, wait=False)
            grip_level += 1
        if code == 311 and value == 1 and grip_level < claw_full_grip:
            claw.run_time(-claw_speed, 4000*(5-grip_level), then=Stop.COAST, wait=False)
            grip_level += 5
        if (code == 305 or code == 310) and value == 1:                                              #Open the grabber
            claw.run_time( claw_speed, 2500, then=Stop.COAST, wait=False)
            grip_level = 0

    if ev_type == 3: # Stick was moved
        if code == 0: # left joystick, turret rotation                                                  #TODO add touch sensor limits
            if value < 118 or value > 138: 
                if (value < 118 and touch_turret_ccw.pressed() == False) or (value > 138 and touch_turret_cw.pressed() == False):
                    turret_rot.run(scale(value, (0,255), (-turret_speed, turret_speed)))
                else: turret_rot.stop()
            else: turret_rot.stop()
        elif turret_rot.speed() != 0:
            if (turret_rot.speed() < 0 and touch_turret_ccw.pressed() == True) or (turret_rot.speed() > 0 and touch_turret_cw.pressed() == True):
                turret_rot.stop()
        if code == 3: # right joystick horizontal, sub boom up and down
            if value < 118 or value > 138: 
                if   value < 118 and sub_boom.angle() >  valve_center_to_end:     sub_boom.reset_angle( valve_center_to_end)
                elif value > 138 and sub_boom.angle() < -valve_center_to_end:     sub_boom.reset_angle(-valve_center_to_end)
                if   value < 118 and sub_boom.angle() < -sub_boom_up_to_down*360: sub_boom.stop()
                elif value > 138 and sub_boom.angle() >  sub_boom_down_to_up*360: sub_boom.stop()
                else: sub_boom.run(scale(value, (0,255), (-nominal_speed_boom_up, nominal_speed_boom_up)))
            else: sub_boom.stop()
        else:
            if sub_boom.angle() < -sub_boom_up_to_down*360 or sub_boom.angle() > sub_boom_down_to_up*360: sub_boom.stop()
        if code == 4: # right stick vertical, main boom up and down
            if value < 118 or value > 138: 
                if   value < 118 and main_boom.angle() >  valve_center_to_end:      main_boom.reset_angle( valve_center_to_end)
                elif value > 138 and main_boom.angle() < -valve_center_to_end:      main_boom.reset_angle(-valve_center_to_end)
                if   value < 118 and main_boom.angle() < -main_boom_up_to_down*360: main_boom.stop()
                elif value > 138 and main_boom.angle() >  main_boom_down_to_up*360: main_boom.stop()
                else: 
                    print(value, main_boom.angle(), main_boom_down_to_up*360)
                    main_boom.run(scale(value, (0,255), (-nominal_speed_boom_up, nominal_speed_boom_up)))
            else: main_boom.stop()
        else:
            if main_boom.angle() < -main_boom_up_to_down*360 or main_boom.angle() > main_boom_down_to_up*360: main_boom.stop()

    event = in_file.read(EVENT_SIZE)
in_file.close()