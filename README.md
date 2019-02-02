# zerobot
Gamepad-controlled Pi Zero W robot with servo-gripper/claw

Prerequisites:
1. Pi Zero W/Pi 3
2. ZeroBorg Quadmotor Driver
3. i2c OLED 128x64 
4. PCA9685 16-channels 12-bit RC servo controller
5. Cheap wireless (usb dongle/bluetooth) joystick/gamepad 

Install dependencies:

Copy and paste the following commands in Terminal:


1. Update your Pi libraries
<code>sudo apt-get -y update && sudo apt-get -y upgrade</code>

2. Download & install ZeroBorg libraries
<code> mkdir ~/zeroborg </code>
  
  cd ~/zeroborg
  
  wget http://www.piborg.org/downloads/zeroborg/examples.zip
  
  unzip examples.zip
  
  chmod +x install.sh
  
  ./install.sh
</code>
