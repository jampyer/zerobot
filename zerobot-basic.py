#!/usr/bin/env python
# coding: UTF-8
# edited by Zidz Feb 15 (Friday)

import RPi.GPIO as GPIO
import time
import os
import sys
import pygame
import ZeroBorg
from time import sleep
import subprocess


driveLeft = 0.0
driveRight = 0.0
axisUpDown = 1                          # Joystick axis to read for up / down position
axisUpDownInverted = False              # Set this to True if up and down appear to be swapped
axisLeftRight = 3                       # Joystick axis to read for left / right position
axisLeftRightInverted = False           # Set this to True if left and right appear to be swapped
buttonResetEpo = 7                      # Joystick button number to perform an EPO reset (Start)
interval = 0.00                         # Time between updates in seconds, smaller responds faster but uses more processor time
buttonA = 0
buttonB = 1
buttonX = 2
buttonY = 3
buttonSelect = 6
buttonHome = 8

# Re-direct our output to standard error, we need to ignore standard out to hide some nasty print statements from pygame
print("starting zerobot")
    
sys.stdout = sys.stderr
# zeroborg variables
ZB = ZeroBorg.ZeroBorg()	# make this global
# Setup the ZeroBorg
ZB.i2cAddress = 0x0a                  # Uncomment and change the value if you have changed the board address
ZB.Init()
if not ZB.foundChip:
    boards = ZeroBorg.ScanForZeroBorg()
    if len(boards) == 0:
        print 'No ZeroBorg found, check you are attached :)'
    else:
        print 'No ZeroBorg at address %02X, but we did find boards:' % (ZB.i2cAddress)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the I2C address change the setup line so it is correct, e.g.'
        print 'ZB.i2cAddress = 0x%02X' % (boards[0])
    sys.exit()
#ZB.SetEpoIgnore(True)        # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
# Ensure the communications failsafe has been enabled!
failsafe = False
for i in range(5):
    ZB.SetCommsFailsafe(True)
    failsafe = ZB.GetCommsFailsafe()
    if failsafe:
        break
if not failsafe:
    print 'Board %02X failed to report in failsafe mode!' % (ZB.i2cAddress)
    sys.exit()
ZB.ResetEpo()

ZB.MotorsOff()
os.environ["SDL_VIDEODRIVER"] = "dummy" # Removes the need to have a GUI window
os.putenv('DISPLAY',':0')
pygame.init()
print 'Waiting for joystick... (press CTRL+C to abort)'              
while True:
    try:
        try:
            pygame.joystick.init() 
            # Attempt to setup the joystick
            if pygame.joystick.get_count() < 1:
                # No joystick attached, toggle the LED
                ZB.SetLed(not ZB.GetLed())
                pygame.joystick.quit()
                time.sleep(0.1)
            else:
                # We have a joystick, attempt to initialise it!
                joystick = pygame.joystick.Joystick(0)
                break
        except pygame.error:
            # Failed to connect to the joystick, toggle the LED
            ZB.SetLed(not ZB.GetLed())
            pygame.joystick.quit()
            time.sleep(0.1)
    except KeyboardInterrupt:
        # CTRL+C exit, give up
        print '\nUser aborted'
        ZB.SetLed(True)
        sys.exit()
print 'Joystick found'
joystick.init()
ZB.SetLed(False)

# main thread (foreground process)
try:
    print("Press Ctrl+C to quit")
    running = True
    hadEvent = False
    upDown = 0.0
    leftRight = 0.0
    # Loop indefinitely
    while running:
        a = 0
        b = 0
        x = 0
        y = 0
        home = 0
        select = 0
        # Get the latest events from the system
        ZB.SetLed(False)
        hadEvent = False
        events = pygame.event.get()
        # Handle each event individually
        for event in events:
            if event.type == pygame.QUIT:
                # User exit
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                # A button on the joystick just got pushed down
                hadEvent = True                    
            elif event.type == pygame.JOYAXISMOTION:
                # A joystick has been moved
                hadEvent = True
            if hadEvent:
                ZB.SetLed(True)
                # Read axis positions (-1 to +1)
                if axisUpDownInverted:
                    upDown = -joystick.get_axis(axisUpDown)
                else:
                    upDown = joystick.get_axis(axisUpDown)
                if axisLeftRightInverted:
                    leftRight = -joystick.get_axis(axisLeftRight)
                else:
                    leftRight = joystick.get_axis(axisLeftRight)
                # Determine the drive power levels
                driveLeft = -upDown
                driveRight = -upDown
                if leftRight < -0.05:
                    # Turning left
                    driveLeft *= 1.0 + (1.5 * leftRight)
                elif leftRight > 0.05:
                    # Turning right
                    driveRight *= 1.0 - (1.5 * leftRight)
                    
                ZB.SetMotor1(driveLeft)
                ZB.SetMotor2(driveLeft)
                ZB.SetMotor3(driveRight)
                ZB.SetMotor4(driveRight)
                
                # Check for button presses
                # get all face buttons input
                if joystick.get_button(buttonA):
                    a = 1
                if joystick.get_button(buttonB):
                    b = 1
                if joystick.get_button(buttonX):
                    x = 1
                if joystick.get_button(buttonY):
                    y = 1
                if joystick.get_button(buttonSelect):
                    select = 1
                    displayOption+=1
                    if displayOption>4:
                        displayOption = 0
        
                # update stream at interval
                print("{:.2f} L ... R {:.2f}    A:{} B:{} X:{} Y:{} select:{} countSel {} home:{}".format(driveLeft,driveRight,a,b,x,y,select,displayOption,home))
               
                # DANGER AREA
                if joystick.get_button(buttonHome):
                    print("WARNING!!!! HOME WAS PRESSED!!! RETURNING TO LAUNCHER")
                    raise KeyboardInterrupt
                time.sleep(interval)
    ZB.MotorsOff()
    print("Shutting down Zerobot")

except KeyboardInterrupt:
    # CTRL+C exit, disable all drives
    ZB.MotorsOff()

print ("Exiting Main Thread. All is good! Goodbye")
