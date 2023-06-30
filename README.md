# instax_api
[![.github/workflows/python-test.yml](https://github.com/jpwsutton/instax_api/actions/workflows/python-test.yml/badge.svg)](https://github.com/jpwsutton/instax_api/actions/workflows/python-test.yml)
[![Coverage Status](https://img.shields.io/coveralls/jpwsutton/instax_api/master.svg)](https://coveralls.io/github/jpwsutton/instax_api?branch=master)

This is a Python Module to interact and print photos to the Fujifilm Instax SP-2 and SP-3 printers.


## Install this library

In order to use this library, you will need to be using Python 3

```
pip3 install instax-api
```


## Usage

**note** - From version 0.7.0 to 0.8.0, I moved away from adding a script to just calling the module from pyton using the `-m` argument.

```
$ python3 -m instax.print --help
usage: instax-print [-h] [-i PIN] [-v {1,2,3}] image

positional arguments:
  image                 The location of the image to print.

optional arguments:
  -h, --help            show this help message and exit
  -i PIN, --pin PIN     The pin code to use, default: 1111.
  -v {1,2,3}, --version {1,2,3}
                        The version of Instax Printer to use (1, 2 or 3).
                        Default is 2 (SP-2).                       
```

### Examples:

 - Printing a Photo to an SP-2 printer: `python3 -m instax.print myPhoto.jpg`
 - Printing a Photo to an SP-3 printer: `python3 -m instax.print myPhoto.jpg -v 3`
 - Printing a Photo to a printer with a pin that is not the default (1111) `python3 -m instax.print myPhoto.jpg -i 1234`

### Hints and tips:
 - Make sure you are connected to the correct wifi network, once the printer is turned on, there will be an SSID / WiFi network available that starts with `INSTAX-` followed by 8 numbers. You'll need to connect to this.
 - If you have a static IP address set up on your computer, you'll need to turn on DHCP before attempting to print, the Instax printer will automatically assign you a new address once you connect.
- Some Unix based operating systems may require you to use sudo in order to access the network.
- The printer will automatically turn itself off after roughly 10 minutes of innactivity.
- The instax.print utility will attempt to automatically rotate the image so that it either is correctly printed in portrait, or landscape with the thick bottom edge of the print on the left. If you wish to print your photos in a specific orientation that differs from this, then it's reccomended that you orient your photo in a tool like GIMP first, then strip out the rotation metadata. Once the rotation metadata has been stripped, the photo will need to be in a portrait orientation relative to the finished print (e.g. thick edge at the bottom). 

## Install Manually

```
git clone https://github.com/jpwsutton/instax_api.git
cd instax_api
python3 setup.py install
```


# ROS wrapper usage

A ROS wrapper has been created to allow sending for printing through ROS. 


## Set-up

Clone the following repository that is used for photo taking and has the service msgs:

```
git clone --branch develop https://github.com/haru-project/haru_photo_activity
```

If you wish to test the full photo taking check the README of this repository.

After cloning this repository, create a python virtualenv and install all the python requirements. Note you can also reuse the existing virtual environment in this repository, venv_instax, but it is recommended to always recreate it the first time.

Note that for integration purposes, you need to do this even if you have done the setup.py install setup above. 


```bash
 python3 -m venv venv_instax > /dev/null
 sudo chown -R $(whoami) venv_instax
 source venv_instax/bin/activate
 pip3 install --upgrade pip
 pip3 install wheel
 pip3 install -r requirements.txt --no-cache-dir 
 deactivate
```

## ROS service printing usage


First make sure the computer is connected to the WIFI network of the Instax device, and check that the following arguments are correct, inside the instax_api_ros.launch file. 

```
    <arg name="host"      default="192.168.0.251"/>
    <arg name="port"      default="8080"/>
    <arg name="pin"      default="1111"/>
    <arg name="timeout"      default="10"/>
```

To test it:

```
roslaunch instax_api_ros instax_api_ros.launch
```

And call the service indicating the directory of the image to be printed:

```
rosservice call /send_photo "photo_dir: '/home/haru/haru_user_photos/haru_img_insta.png'" 
```

Inside the Telegram app, the service will be called once the user is happy with the photo. Check the
interfacing in:

https://github.com/haru-project/idmind-tabletop-telegram-bot/blob/feature/take_picture/src/utils/ros_client.py


