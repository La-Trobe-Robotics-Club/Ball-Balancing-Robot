'''
###Linux COMMANDS to setup the environment:

sudo apt-get update
sudo apt-get upgrade

###verify python3 and openCV are installed:

python3 --version
sudo apt-get install python3-pip
sudo pip3 install opencv-python-headless opencv-contrib-python-headless
sudo apt-get install python3-serial

nano test.py
    import cv2
    import serial
    print(cv2.__version__)
    print(serial.__version__)

python3 test.py
nano balltracker.py
    import cv2
    import serial


'''