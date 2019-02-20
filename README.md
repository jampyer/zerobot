# zerobot
Gamepad-controlled Pi Zero W robot with servo-gripper/claw
![joystick](https://github.com/rtxsc/zerobot/blob/master/images/2A944D83-D702-4C6A-9C70-55236CA0A1AE.jpeg)

##### Prerequisites:
    1. Pi Zero W/Pi 3
    2. ZeroBorg Quadmotor Driver
    3. i2c OLED 128x64 
    4. PCA9685 16-channels 12-bit RC servo controller
    5. Cheap wireless (usb dongle/bluetooth) joystick/gamepad 

# Installation:

Copy and paste the following commands in Terminal:

### 1. Update your Pi libraries

`sudo apt-get -y update && sudo apt-get -y upgrade`

### 2. Download & install ZeroBorg libraries

``` mkdir ~/zeroborg
    cd ~/zeroborg
    wget http://www.piborg.org/downloads/zeroborg/examples.zip
    unzip examples.zip 
    chmod +x install.sh
    ./install.sh
```
### 3. Install Adafruit PCA9685 library

``` 
sudo apt-get install git build-essential python-dev
cd ~
git clone https://github.com/adafruit/Adafruit_Python_PCA9685.git
cd Adafruit_Python_PCA9685
sudo python setup.py install

```

### 4. Install Adafruit SSD1306 library

```
sudo python -m pip install --upgrade pip setuptools wheel
cd ~
git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git
cd Adafruit_Python_SSD1306
sudo python setup.py install

```

### 5. Install INA219 library (optional)

```
sudo pip install pi-ina219

```
















