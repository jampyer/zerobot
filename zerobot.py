#!/usr/bin/env python
# coding: UTF-8
# edited by Zidz Feb 15 (Friday)

import RPi.GPIO as GPIO
import time
import os
import sys
import pygame
import ZeroBorg
import Adafruit_PCA9685
from gpiozero import RGBLED,DistanceSensor,Buzzer
from ina219 import INA219
from ina219 import DeviceRangeError
from time import sleep

from PIL import Image,ImageDraw,ImageFont
import subprocess
from time import sleep
import threading

# INA219 variables
SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 0.2
ibus = 0.0

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

# HCSR04 variables
frontProximity = 0.0

# servo drive PCA9685 variables
tilt = 0    # default gripper tilt
claw = 1    # default claw 

tilt_down = 300  # Min pulse length out of 4096
tilt_up = 400  # Max pulse length out of 4096
claw_open = 200
claw_close = 400

delay = 0.01
step = 10
steptilt = 1
pos = tilt_up
servo_cmd = " "
logOnce = 0
displayOption = 0

logOnceC =0
posC = claw_close

# gpiozero RGBLED variables and function
buz = Buzzer(16)


def beep_shorter():
    buz.on()
    sleep(0.25)
    buz.off()
    sleep(0.25)


def beep_short():
    buz.on()
    sleep(0.5)
    buz.off()
    sleep(0.5)
    
def beep_long():
    buz.on()
    sleep(0.5)
    buz.off()
    sleep(0.5)
    buz.on()
    sleep(0.5)
    buz.off()
    sleep(0.5)
    
rgb = RGBLED(19,26,13)
def ongreen():
    rgb.color = (0,1,0)
def onred():
    rgb.color = (1,0,0)
def onblue():
    rgb.color = (0,0,1)

def readINA219():
    ina = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x44)
    ina.configure(ina.RANGE_16V)
    global ibus
    # to reduce burden on CPU we only measure current 
    try:
        #vbus = ina.voltage()
        ibus = ina.current()
        #power = ina.power()
        #vshun = ina.shunt_voltage()
        print("measuring current:{:.2f}mA".format(ibus))
        #print("busVolt:{:.2f}V busAmp:{:.2f}mA Pow:{:.2f}mW shunVolt:{:.2f}mV\n".format(vbus,ibus,power,vshun))
    except DeviceRangeError as e:
        # Current out of device range with specified shunt resistor
        print(e)



# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def _close():
    printOnceC = 0
    global posC
    global logOnceC
    global clawIsClose
    global claw_close
    stopPos = 0
    tooHighCurrent = 1000

    for i in range (claw_open,claw_close,+step):
        ongreen()
        pwm.set_pwm(claw, 0, i)
        posC = i
        readINA219()
        # here we limit the current to 50mA
        # if exceed, stop the servo at the current position
        if (ibus > tooHighCurrent):
        	onred()
        	print("high current spike detected= {:.2f}mA".format(ibus))
        	print("overcurrent by {:.2f}mA detected!".format(ibus-tooHighCurrent))
        	stopPos = posC
        	claw_close = stopPos
        	print("close position reduced to {}\n".format(stopPos))
        	break
        if(printOnceC!=0):
            pass
        else:
            print "closing claw"
        printOnceC+=1
        time.sleep(delay)
    clawIsClose = True
    logOnceC = 0

def _open():
    beep_shorter()
    ongreen()
    printOnceC = 0
    global posC
    global logOnceC
    global clawIsClose
    global claw_close   # to retrieve current close position (may change due to spike)

    for i in range (claw_close,claw_open,-step):
        pwm.set_pwm(claw, 0, i)
        posC = i
        print("many opening")
        if(printOnceC!=0):
            pass
        else:
            print "opening claw"
        printOnceC+=1
        time.sleep(delay)
    claw_close = 400 # reset back close position after open 
    clawIsClose = False
    logOnceC = 0

def go_down():
    """" after execution, gripper status is DOWN """
    printOnce = 0
    global pos
    global logOnce
    global gripperIsDown
    for i in range (tilt_up,tilt_down,-steptilt):
        pwm.set_pwm(tilt, 0, i)
        pos = i
        if(printOnce!=0):
            pass
        else:
            print "gripper is moving down"
        printOnce+=1
        time.sleep(delay)
    gripperIsDown = True
    logOnce = 0

def go_up():
    """" after execution, gripper status is UP """
    global ibus
    global pos
    global logOnce
    printOnce = 0
    stopPos = 0
    global gripperIsDown

    for i in range (tilt_down,tilt_up,steptilt):
        pwm.set_pwm(tilt, 0, i)
        pos = i
        if(printOnce!=0):
            pass
        else:
            print "gripper is moving UP"
        printOnce+=1
        time.sleep(delay)
    gripperIsDown = False
    logOnce =0

def checkout():
    if not clawIsClose:
        _close()
    else:
        pass
    if not gripperIsDown:
        go_down()
    else:
        pass    
    beep_long()
"""    
def reset_claw():
    global pos
    global logOnce
    printOnce = 0
    global gripperIsDown
    
    for i in range (pos,tilt_up,step):
        pwm.set_pwm(tilt, 0, i)
        if(printOnce!=0):
            pass
        else:
            print "resetting"
        #print("reseting Angle:{} pos to max 500:{}".format(i,tilt,i))
        time.sleep(delay)
        printOnce+=1
    gripperIsDown = False
    logOnce = 0
"""    

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
            onblue()
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
                beep_short()
                ongreen()
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

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)


# main thread (foreground process)
try:
    print("Press Ctrl+C to quit")
    running = True
    hadEvent = False
    upDown = 0.0
    leftRight = 0.0
    clawIsClose = True      # should be close at start
    gripperIsDown = True    # should be down at start
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

                
                if (a == 1):
                    servo_cmd = "tilt_down_cmd"
                if (b == 1):
                    servo_cmd = "tilt_up_cmd"
                if (x == 1):
                    servo_cmd = "close_cmd"
                if (y == 1):
                    servo_cmd = "open_cmd"
                
                if(servo_cmd == "tilt_down_cmd"):
                    if not gripperIsDown:
                        go_down()
                    else:
                        pass
                if(servo_cmd == "tilt_up_cmd"):
                    if gripperIsDown:
                        go_up()
                    else:
                        pass
                        
                if(servo_cmd == "close_cmd"):
                    if not clawIsClose:
                        _close()
                    else:
                        pass
                if(servo_cmd == "open_cmd"):
                    if clawIsClose:
                        _open()
                    else:
                        pass
                """
                if(servo_cmd == "reset"):
                    if gripperIsDown:
                        go_down()
                    else:
                        pass
                """
                # update stream at interval
                print("{:.2f} L ... R {:.2f}    A:{} B:{} X:{} Y:{} select:{} countSel {} home:{}".format(driveLeft,driveRight,a,b,x,y,select,displayOption,home))
                
                
                # DANGER AREA
                if joystick.get_button(buttonHome):
                    home = 1
                    
                if home:
                    print("WARNING!!!! HOME WAS PRESSED!!! RETURNING TO LAUNCHER")
                    checkout()
                    
                    raise KeyboardInterrupt
                    
                time.sleep(interval)
    ZB.MotorsOff()
    print("Shutting down Zerobot")
except IOError:
    print("FUCK THIS ERROR")
    pass
    #raise KeyboardInterrupt

except KeyboardInterrupt:
    # CTRL+C exit, disable all drives
    ZB.MotorsOff()
    # quick check ! make sure gripper is close and in down position before shutting off
    checkout()
                    
print ("Exiting Main Thread. All is good! Goodbye")
