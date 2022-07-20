#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2021 Bitcraze AB
#
#  AI-deck demo
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License along with
#  this program; if not, write to the Free Software Foundation, Inc., 51
#  Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
#  Demo for showing streamed JPEG images from the AI-deck example.
#
#  By default this demo connects to the IP of the AI-deck example when in
#  Access point mode.
#
#  The demo works by opening a socket to the AI-deck, downloads a stream of
#  JPEG images and looks for start/end-of-frame for the streamed JPEG images.
#  Once an image has been fully downloaded it's rendered in the UI.
#
#  Note that the demo firmware is continously streaming JPEG files so a single
#  JPEG image is taken from the stream using the JPEG start-of-frame (0xFF 0xD8)
#  and the end-of-frame (0xFF 0xD9).



###================================
# Modified by Miguel Granero. 2022
#   - ROS Package created
#   - Modification to functionality to adapt it to a better ros usage
#   - Image publisher
###================================


import argparse
import time
import socket,os,struct, time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class aideckPublisher(Node):
    def __init__(self):
        super().__init__('aideck_stream_publisher')

        # Args for setting IP/port of AI-deck. Default settings are for when
        # AI-deck is in AP mode.
        self.declare_parameter('ip','192.168.4.1')
        deck_ip = self.get_parameter('ip').value
        self.declare_parameter('port',5000)
        deck_port = self.get_parameter('port').value
        self.declare_parameter('save_flag',False)
        self.declare_parameter('show_flag',False)
        
        
        self.publisher_ = self.create_publisher(Image, 'aideck/image', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((deck_ip, deck_port))
        print("Socket connected")

        self.br = CvBridge()

        self.start = time.time()
        self.count = 0


    def getImage(self, client_socket):
        imgdata = None
        data_buffer = bytearray()
        # First get the info
        packetInfoRaw = rx_bytes(4, client_socket)
        #print(packetInfoRaw)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
        #print("Length is {}".format(length))
        #print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
        #print("Function is 0x{:02X}".format(function))

        imgHeader = rx_bytes(length - 2, client_socket)
        #print(imgHeader)
        #print("Length of data is {}".format(len(imgHeader)))
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        imgs = None

        if magic == 0xBC:
            #print("Magic is good")
            #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
            #print("Image format is {}".format(format))
            #print("Image size is {} bytes".format(size))

            # Now we start rx the image, this will be split up in packages of some size
            imgStream = bytearray()

            while len(imgStream) < size:
                packetInfoRaw = rx_bytes(4, client_socket)
                [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = rx_bytes(length - 2, client_socket)
                imgStream.extend(chunk)
            
            self.count = self.count + 1
            meanTimePerImage = (time.time()-self.start) / self.count
            print("{}".format(meanTimePerImage))
            print("{}".format(1/meanTimePerImage))

            if format == 0:
                bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
                bayer_img.shape = (244, 324)
                color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                
                if self.get_parameter('save_flag').value:
                    cv2.imwrite(f"stream_out/raw/img_{self.count:06d}.png", bayer_img)
                    cv2.imwrite(f"stream_out/debayer/img_{self.count:06d}.png", color_img)
                if self.get_parameter('show_flag').value:
                    cv2.imshow('Raw', bayer_img)
                    cv2.imshow('Color', color_img)
                    cv2.waitKey(1)
                imgs = [bayer_img,color_img]
            else:
                if self.get_parameter('save_flag').value:
                    with open("img.jpeg", "wb") as f:
                        f.write(imgStream)
                nparr = np.frombuffer(imgStream, np.uint8)
                decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                if self.get_parameter('show_flag').value:
                    cv2.imshow('JPEG', decoded)
                    cv2.waitKey(1)
                imgs = [decoded]

        return format,imgs

    def timer_callback(self):
        msg = Image()
        format, imgs = self.getImage(self.client_socket)

        if imgs is not None:
            #self.get_logger().info('Publishing: "%s"' % self.i)
            img = imgs[-1]
            msg = self.br.cv2_to_imgmsg(img)

            self.publisher_.publish(msg)
            self.i += 1


def rx_bytes(size, client_socket):
    data = bytearray()
    while len(data) < size:
        data.extend(client_socket.recv(size-len(data)))
    return data



def main(args=None):

    rclpy.init(args=args)
    stream_pub = aideckPublisher()
    rclpy.spin(stream_pub)

    stream_pub.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
