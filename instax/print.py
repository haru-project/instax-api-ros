#!/usr/bin/env python3
"""Instax SP Print Script.

Author: James Sutton 2017 - jsutton.co.uk

This can be used to print an image to a Fujifilm Instax SP-2 printer.
Parameters:
 - JSON Log File (Default ddmmyy-hhmmss.json)
 - Image to print
 - Port (Default 8080)
 - Host (Default 192.168.0.251)

"""
import argparse
import datetime
import sys

from loguru import logger

from instax.instaxImage import InstaxImage
from instax.sp2 import SP2
from instax.sp3 import SP3

#Dependencies for conversion to ROS service

import rospy
import haru_photo_activity.srv



def printPrinterInfo(info):
    """Log Printer information"""
    logger.info("Model: %s" % info["model"])
    logger.info("Firmware: %s" % info["version"]["firmware"])
    logger.info("Battery State: %s" % info["battery"])
    logger.info("Prints Remaining: %d" % info["printCount"])
    logger.info("Total Lifetime Prints: %d" % info["count"])
    logger.info("")


# https://gist.github.com/vladignatyev/06860ec2040cb497f0f3
def printProgress(count, total, status=""):
    # logger.info(status)
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))
    percents = round(100.0 * count / float(total), 1)
    bar = "=" * filled_len + "-" * (bar_len - filled_len)
    sys.stdout.write("[{}] {}{} ...{}\r".format(bar, percents, "%", status))
    sys.stdout.flush()  # As suggested by Rom Ruben (see: http://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console/27871113#comment50529068_27871113)

class PrintInstax(object):
    #Initialize ROS server to send for printing
    def __init__(self):
        self.print_photo_server = rospy.Service(
            'send_photo', haru_photo_activity.srv.TakePhoto, self.print_image)
        host = rospy.get_param("~host", "192.168.0.251")
        port = rospy.get_param("~port", 8080)
        pin = rospy.get_param("~pin", 1111)
        timeout = rospy.get_param("~timeout", 10)
        self.myInstax = SP3(ip=host, port=port, pinCode=pin, timeout=timeout)
    def print_image(self, srvs):
        logger.info("Connecting to Printer.")
        info = self.myInstax.getPrinterInformation()
        printPrinterInfo(info)

        logger.info("Printing Image: %s" % srvs.photo_dir)
        # Initialize The Instax Image
        instaxImage = InstaxImage(type=3) #restrict to type 3 Instax printer
        instaxImage.loadImage(srvs.photo_dir)
        instaxImage.convertImage()
        # Save a copy of the converted bitmap
        # instaxImage.saveImage("test.bmp")
        # Preview the image that is about to print
        # instaxImage.previewImage()
        encodedImage = instaxImage.encodeImage()
        self.myInstax.printPhoto(encodedImage, printProgress)
        logger.info("Thank you for using instax-print!")
        return haru_photo_activity.srv.TakePhotoResponse(True)


if __name__ == "__main__":
    rospy.init_node('instax_printer')
    server = PrintInstax()
    rospy.spin()
