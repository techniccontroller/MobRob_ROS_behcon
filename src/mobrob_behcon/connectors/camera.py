#!/usr/bin/env python

import socket
import base64
import io
from io import BytesIO
import numpy as np
from PIL import Image
from PIL import ImageFile
ImageFile.LOAD_TRUNCATED_IMAGES = True
import cv2

class CameraTCP(object):
    """
    The class CameraTCP

    Represents a connector to the Camera on the robot.
    Provides a function to get a image from camera.
    """
    
    BUFFER_SIZE = 16
    MESSAGE = b'getNewFrame'

    def __init__(self, ip_address, port):
        """
        constructor

        :param ip_address: ip address of camera server
        :param port: port of camera server
        :type ip_address: string
        :type port: int 
        """
        self.ip_address = ip_address
        self.port = port

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.ip_address, self.port))
    

    def get_frame(self):
        """
        Get new frame from camera

        :return: returns current frame from camera
        """
        self.socket.send(self.MESSAGE)
        data = self.socket.recv(self.BUFFER_SIZE)
        size = int(data)
        
        data = CameraTCP.recvall(self.socket, size)

        frame = CameraTCP.stringToRGB(data) 
        return frame
    
    @staticmethod
    def stringToRGB(base64_string):
        """
        Convert base64 encoded image to OpenCV image

        :param base64_string: image encoded as base64 string
        :type base64_string: string
        :return: returns image as OpenCV image
        """
        image = base64.b64decode(base64_string)
        image = BytesIO(image)
        image = Image.open(image)
        image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        return image
    
    @staticmethod
    def recvall(sock, n):
        """
        Helper function to recv n bytes or return None if EOF is hit

        :param sock: socket
        :type sock: TCPServer
        :param n: number of bytes
        :type n: int
        :return: returns data as bytearray
        """
        data = bytearray()
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data