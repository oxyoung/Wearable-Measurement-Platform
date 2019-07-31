"""
This code builds the connection between Raspberry Pi 3B+ and Arduino Due and controls data acquisition
and processing function of Raspberry Pi 3B+.
"""

import BlynkLib
import time
import serial
import numpy as np
import re
import math
import socket
import csv
from Filters import FiltersCombination

"""
System Parameter Initialization
"""
EMG_normalization = False
IMU_normalization = False
SERVER = 'local'

"""
Blynk Initialization
"""
# Set the unique Blynk project code, if local server is used, set corresponding IP address
if SERVER == 'cloud':
    blynk = BlynkLib.Blynk('88abb8a4773b4e60bb513f5605cfc0bb')
elif SERVER == 'local':
    blynk = BlynkLib.Blynk('1cbed7e513e84fd0b17393c48e94eec1', server='192.168.253.1', port=80)

# Serial connection port and baudrate initialization. Serial port can be found from 'ls /dev/tty/ACM*' in RP 3B+
# Command Terminal.
try:
    ser = serial.Serial("/dev/ttyACM1", 2000000)
except serial.SerialException as e:
    ser = serial.Serial("/dev/ttyACM0", 2000000)
    pass
ser.baudrate = 2000000

# Blynk virtual Pin setting
EMG1_VPIN = 1
EMG2_VPIN = 2
IMUX_VPIN = 4
IMUY_VPIN = 5
IMUZ_VPIN = 6
IMU_BUTTON = 7
EMG_BUTTON = 8
START_BUTTON = 3

# threshold1 = 800
# threshold2 = 800
# filters1 = FiltersCombination(1000)
# filters2 = FiltersCombination(1000)

read_ser = 'none'

eMGinOneSec1 = list()
eMGinOneSec2 = list()

max_EMG1 = 0
max_EMG2 = 0
imuX_initial = list()
imuY_initial = list()
imuZ_initial = list()
EMG1_initial = list()
EMG2_initial = list()
EMG1max_initial = list()
EMG2max_initial = list()

# Create sensor data file
filename = 'sensor_data_{:.0f}.csv'.format(time.time())
sensor_file = open(filename, mode='wb')
sensor_writer = csv.writer(sensor_file, delimiter=',')

"""
Define start button function
"""
@blynk.VIRTUAL_WRITE(START_BUTTON)
def v3_write_handler(value):
    global t_last
    if value[0] == u'1':
        print('Task Start!'.format(value))
        start = time.time()
        t_last = time.time()
        averageEMG1 = 0;
        averageEMG2 = 0;
        while True:
            if ser.inWaiting():
                read_ser = ser.readline()
                sensor_values = list(map(float, read_ser.split()))
                if len(sensor_values) == 2:
                    eMGinOneSec1.append(sensor_values[0])
                    eMGinOneSec2.append(sensor_values[1])
                # Should change according to Arduino due serial print
                elif len(sensor_values) == 3:
                    if IMU_normalization is True:
                        sensor_values[2] = get_orientation_percent(sensor_values[2], min_y, max_y)
                    y_reading = sensor_values[2]
                    eMGinOneSec1.append(sensor_values[0])
                    eMGinOneSec2.append(sensor_values[1])
                # Write sensor value to CSV file
                sensor_writer.writerow(sensor_values)
            # Every 0.1 second, write sensor value to Blynk interface
            if (time.time() - t_last) >= 0.1:
                if len(eMGinOneSec1) == 0:
                    averageEMG1 = 0
                elif len(eMGinOneSec2) == 0:
                    averageEMG2 = 0
                else:
                    averageEMG1 = sum(eMGinOneSec1) / len(eMGinOneSec1)
                    averageEMG2 = sum(eMGinOneSec2) / len(eMGinOneSec2)
                if EMG_normalization is True:
                    averageEMG1, averageEMG2 = emg_normalization(averageEMG1, averageEMG2)

                blynk.virtual_write(EMG1_VPIN, averageEMG1)
                blynk.virtual_write(EMG2_VPIN, averageEMG2)
                blynk.virtual_write(IMUY_VPIN, y_reading)

                # clear all element in emg list
                del eMGinOneSec1[:]
                del eMGinOneSec2[:]

                t_last = time.time()
            # Used for testing
            # if time.time() - start >= 35:
            #    sensor_writer.writerow(str(time.time()-start))
            #    quit()

    else:
        print('Current slider value: {}. Task Over!'.format(value))

"""
IMU normalization process
"""
@blynk.VIRTUAL_WRITE(IMU_BUTTON)
def v7_write_handler(value):
    if value[0] == u'1':
        global IMU_normalization
        global min_y, max_y, mid_x, min_x, max_x
        IMU_normalization = True
        min_y, max_y = 0, 0
        start_time = time.time()

        while time.time() - start_time < 10:
            read_ser = ser.readline()
            print(read_ser)
            sensor_values = list(map(float, read_ser.split()))
            print(len(sensor_values))
            if len(sensor_values) == 3:
                imuY_initial.append(sensor_values[2])

            if len(imuY_initial) > 1:
                min_y, max_y = get_boundary(imuY_initial)
            print(min_y)
            print(max_y)
        del imuY_initial[:]
    print('IMU Normalization Over')

"""
EMG normalization process
"""
@blynk.VIRTUAL_WRITE(EMG_BUTTON)
def v8_write_handler(value):
    if value[0] == u'1':
        global EMG_normalization
        global max_EMG1, max_EMG2
        EMG_normalization = True
        start_time = time.time()
        max_EMG1, max_EMG2 = 0, 0
        t_last = time.time()
        average1, average2 = 0, 0
        while True:
            if ser.inWaiting():
                read_ser = ser.readline()
                sensor_values = list(map(float, read_ser.split()))
                if len(sensor_values) > 1:
                    EMG1_initial.append(sensor_values[0])
                    EMG2_initial.append(sensor_values[1])
            if (time.time() - t_last) >= 0.3:
                # if no serial input assign a invalid number
                if len(EMG1_initial) == 0:
                    average1 = 1000000
                elif len(EMG2_initial) == 0:
                    average2 = 1000000
                else:
                    average1 = sum(EMG1_initial) / len(EMG1_initial)
                    average2 = sum(EMG2_initial) / len(EMG2_initial)
                if average1 < 1000 and average2 < 1000:
                    del EMG1_initial[:]
                    del EMG2_initial[:]
                    break
                t_last = time.time()
            print(average1, average2)
        print('start collecting maximal EMG value')
        while time.time() - start_time < 10:
            if ser.inWaiting():
                read_ser = ser.readline()
                sensor_values = list(map(float, read_ser.split()))
                if len(sensor_values) == 2 or 3:
                    EMG1_initial.append(sensor_values[0])
                    EMG2_initial.append(sensor_values[1])
            if (time.time() - t_last) >= 0.1:
                if len(EMG1_initial) == 0:
                    average1 = 0
                elif len(EMG2_initial) == 0:
                    average2 = 0
                else:
                    average1 = sum(EMG1_initial) / len(EMG1_initial)
                    average2 = sum(EMG2_initial) / len(EMG2_initial)
                EMG1max_initial.append(average1)
                EMG2max_initial.append(average2)
                t_last = time.time()
                del EMG1_initial[:]
                del EMG2_initial[:]
            max_EMG1 = max(EMG1max_initial)
            max_EMG2 = max(EMG2max_initial)
            print(max_EMG1)
            print(max_EMG2)
        del EMG1_initial[:]
        del EMG2_initial[:]
        del EMG1max_initial[:]
        del EMG2max_initial[:]
    print('EMG Normalization Over')

def get_boundary(orientation_list):
    # Take the first n data point then average them as the lower bound.
    start_value = max(orientation_list)
    end_value = min(orientation_list)
    return start_value, end_value

def get_orientation_percent(input_value, s_value, e_value):
    output = -(input_value - s_value)/(s_value - e_value)
    if output < 0:
        output = 0
    elif output > 1:
        output = 1
    return output*100

def emg_filters(sensor_values, filters1, filters2):
    dataAfterFilter1 = filters1.EMGfilter_update(sensor_values[0])
    dataAfterFilter2 = filters2.EMGfilter_update(sensor_values[1])
    envelope1 = math.pow(dataAfterFilter1, 2)
    envelope2 = math.pow(dataAfterFilter2, 2)
    # print(sensor_values[0], sensor_values[1])
    if envelope1 > threshold1:
        sensor_values1 = envelope1
    else:
        sensor_values1 = 0
    if envelope2 > threshold2:
        sensor_values2 = envelope2
    else:
        sensor_values2 = 0
    return sensor_values1, sensor_values2


def emg_normalization(sensor_values1, sensor_values2):
    sensor_values1 = (sensor_values1 / max_EMG1)
    sensor_values2 = (sensor_values2 / max_EMG2)
    if sensor_values1 > 1:
        sensor_values1 = 1
    if sensor_values2 > 1:
        sensor_values2 = 1
    return sensor_values1*100, sensor_values2*100

"""
Blynk re-connection
"""
def blynk_connected():
    print("Updating all values from the server...")
    blynk.disconnect()
    blynk.connect()
    print("Connected to server")

if __name__ == '__main__':
    while True:
        try:
            blynk.run()
        except socket.error as e:
            print('Catch socket error {}'.format(e))
            blynk_connected()
            pass
        except ValueError as e:
            print('Catch value error {}'.format(e))
            blynk_connected()
            pass
        except IOError as e:
            if e.Errno == Errno.EPIPE:
                print('EPIPE error {}'.format(e))
                blynk_connected()
            else:
                print("Unexpected error:", sys.exc_info()[0])
                raise




