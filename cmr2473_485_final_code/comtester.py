# -*- coding: utf-8 -*-
"""
Created on Sun Nov 21 23:22:37 2021

@author: Creck
"""


import serial
import time

serialPort2 = serial.Serial(port = "COM15", baudrate = 9600,bytesize=8,timeout=5000,stopbits=serial.STOPBITS_ONE)

output = open("SerialOut.txt",'w')
print("ready")
serialString2 = ""
i = 0

while(i<100*60):

    # Wait until there is data waiting in the serial buffer
    if(serialPort2.in_waiting > 0):

        # Read data out of the buffer until a carraige return / new line is found
        serialString2 = serialPort2.readline()

        # Print the contents of the serial data
        print(serialString2.decode('Ascii'))
        
        output.write(serialString2.decode('Ascii'))
    time.sleep(0.01)
    i = i+1
output.close()
