#!/usr/bin/python
# simple script to launch multiple python scripts of choice 
# this script can be run in the background at boot using crontab

import time
import sys
import os
import pygame
import subprocess

buttonSelect = 6    # to select play mode
buttonStart = 7     # to (Start)
buttonHome = 8      # to return to launcher
modeCount = 3       # set number of python scripts you wanna run

print("-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_")
print("-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_")
print("-_-_-_-_-_-_-_-_-_-_-_-_ Python Script Launcher -_-_-_-_-_-_-_-_-_-_-_-_")
print("-_-_-_-_-_-_ There are {} Python scripts in this directory -_-_-_-_-_-_-".format(modeCount))
print("-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_")
print("-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_")


os.environ["SDL_VIDEODRIVER"] = "dummy" # Removes the need to have a GUI window
os.putenv('DISPLAY',':0')
pygame.init()
print ('Launcher: wait for joystick')              
while True:
    try:
        try:
            pygame.joystick.init() 
            # Attempt to setup the joystick
            if pygame.joystick.get_count() < 1:
                # No joystick attached, toggle the LED
                pygame.joystick.quit()
                time.sleep(0.1)
            else:
                # We have a joystick, attempt to initialise it!
                joystick = pygame.joystick.Joystick(0)
                break
        except pygame.error:
            # Failed to connect to the joystick, toggle the LED
            pygame.joystick.quit()
            time.sleep(0.1)
    except KeyboardInterrupt:
        # CTRL+C exit, give up
        print ('\nLauncher: user aborted')              
        sys.exit()
print ('Launcher: joystick found! hoorayy')              
joystick.init()
# global start declaration is COMPULSORY to avoid problem when returning back to launcher
global start
global runOnce
try:
    print("Launcher: Press Ctrl+C to quit")
    running = True
    hadEvent = False
    iteration = 0
    mode = 0
    runOnce = 0
    # Loop indefinitely
    while running:
        home   = 0
        start  = 0
        select = 0
        # Get the latest events from the system
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
            if hadEvent:
                iteration+=1
                if joystick.get_button(buttonSelect):
                    select = 1
                if joystick.get_button(buttonStart):
                    start = 1
                if joystick.get_button(buttonHome):
                    home = 1
                    start = 0     
                
                if start:
                    if mode is 3 and start is not 0:
                        """
                        if runOnce > 1:
                            start = 0
                            runOnce = 0
                            subprocess.call(['python', 'zerobot.py'])
                        else:
                        """
                        runOnce+=1
                        print("Starting zerobot.py")
                        subprocess.call(['python', 'zerobot.py'])
                    if mode is 1:
                        print("Starting codeone.py")
                        subprocess.call(['python', 'codeone.py'])
                    elif mode is 2:
                        print("Starting codetwo.py")
                        subprocess.call(['python', 'codetwo.py'])
                    else:
                        print("Selected mode not exist!")
                        
                    print("\n\n\nEXITED!!!!!!. press Start to re-initialize\n\n\n")
                
                if select:
                    mode += 1
                    if mode > modeCount:
                        mode = 0
    
            os.system('clear')
            print("select:{} start:{} home:{} iteration:{} mode:{}\n\n".format(select,start,home,iteration,mode))
            print("Mode 1: Codeone | Mode 2: Codetwo")    
            print("Press Select button to change mode | Home to exit to Launcher\n")
                 
                
        time.sleep(0.01) # while loop delay interval
except KeyboardInterrupt:
    print("Terminating Launcher Playground")
