import os
#from termios import NOFLSH
import time
import datetime
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pylab import *

COM_PORT = 'COM10'

dirpath = os.getcwd() + '\\result_files'

#helper function to read all bytes
def read_all_lines(port):
    port.reset_input_buffer()
    """Read all characters on the serial port and return them."""
    if not port.timeout:
        raise TypeError('Port needs to have a timeout set!')

    read_buffer = b''
    
    while True:
        # Read in chunks. Each chunk will wait as long as specified by
        # timeout. Increase chunk_size to fail quicker
        byte_chunk = port.readline()
        read_buffer += byte_chunk
        #print(byte_chunk.decode('utf-8'), end = '')
        if not len(byte_chunk) >= 1:
            break

    return read_buffer


#Get the COM Ports
ser = serial.Serial(COM_PORT, 115200,timeout=0.5,inter_byte_timeout=0.1)
#ser.set_buffer_size(rx_size = 12800, tx_size = 12800)

#Wavelength Calculation
pix=arange(1,289)
#Calibration Values
A_0=3.065154524e+2; B_1=2.724638729; B_2=-1.464056198e-03
B_3=-5.435494295e-06; B_4=1.775337033e-09; B_5=1.465866767e-11

nm=A_0+B_1*pix+B_2*pix**2+B_3*pix**3+B_4*pix**4+B_5*pix**5

import numpy as np
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

print("Wavelengths len: " + str(len(nm)))
print("Nearest green @532nm: " + str(nm[find_nearest(nm,532)]))

def wait_for_data():
    ser.reset_input_buffer()
    #read data from serial port
    serdata = read_all_lines(ser).decode("utf-8")
    global ydata
    for serdataline in serdata.split('\n'):
        #print(serdataline)
        if '<data>' in serdataline:
            data_str = serdataline.split(",")
            remove_tag = data_str.pop(0)
            remove_tag = data_str.pop()
            data = [int(numeric_string) for numeric_string in data_str]
            #print(str(data))
            ydata = data
            print("Spec @532nm: "+str(data[find_nearest(nm,532)]))
        if '<int_time_us>' in serdataline:
            data_str = serdataline.split(",")
            print("Int_time: "+data_str[1]+"us")
        if '<LTA>' in serdataline:
            data_str = serdataline.split(",")
            print("LTA: "+data_str[1]+" "+data_str[2]+" "+data_str[3]+" "+data_str[4]+" "+data_str[5])
        if '<WTA>' in serdataline:
            data_str = serdataline.split(",")
            print("WTA: "+data_str[1]+" "+data_str[2]+" "+data_str[3]+" "+data_str[4]+" "+data_str[5])

#Prepare Plot
xdata = nm
ydata = zeros(len(nm))
plt.ion()

figure, ax = plt.subplots(figsize=(8,6))
line1, = ax.plot(xdata, ydata)

#Do the Continous Reading and Ploting
while(True):
    line1.set_xdata(xdata)
    new_y = wait_for_data()
    
    if (ydata is not None):
        #Draw SPectrogram
        line1.set_ydata(ydata)
        plt.ylim(min(ydata),max(ydata))
        #Update Drawing
        figure.canvas.draw()        
        figure.canvas.flush_events()        
            

    
#ser.close()
            
