# EC545-ROSMASTER-X3-CAR

## Program car:
- connect hdmi cord from car to monitor and keyboard + mouse to car
- when on desktop, make sure the car is connected to BU Guest network

## Get src files onto car:
pull from cloned repo: https://github.com/MichaelSalak/EC545-ROSMASTER-X3-CAR.git

## Run random_drive.py
- open cmd, get to git repo in /ec545-2026
- get the python3 command typed out / ready before pressing enter
- unplug mouse and monitor
- then press enter, quickly unplug keyboard, and put car on floor within 15s
- wait for car to stop moving for program to end 

## Other car info from cmd:
- "hostname -I" --> 10.193.102.38  (robot ip address)
- whoami        --> jetson         (username)
- hostname -I   --> 10.193.102.38  (car ip address)
- ls /opt/ros   --> melodic        (running ROS 1)
- python version: 3.6.9
