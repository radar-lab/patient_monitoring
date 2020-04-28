#!/usr/bin/env python
# Author: Feng Jin

import argparse
import rospy
import rosbag
import numpy as np
import math
import matplotlib.pyplot as plt
from micro_doppler_pkg.msg import MicroDoppler
from ti_mmwave_rospkg.msg import RadarScan
import os

class micro_doppler_signature_proc:
    def __init__(self):
        self.bagsrcdir  = '/home/ece561/rosbag/SevaTraining/training_bag/' 
        self.csvdir     = '/home/ece561/rosbag/SevaTraining/training_bin/'     

    def save_to_csv(self):
        filecnt = 0
        for file in os.listdir(self.bagsrcdir):
            if file.endswith(".bag"):
                filecnt += 1
                print('Loading ' + file + '...')
                label = int(file[-5])
                bag = rosbag.Bag(self.bagsrcdir + file)
                csv_name_x = self.csvdir + 'x_' + file[:-4]
                csv_name_y = self.csvdir + 'y_' + file[:-4]
                # print(bag)
                # print(file)
                # print(label)
                # print(csv_name_x)
                # print(csv_name_y)
                # print('...........................')
                for msg in bag.read_messages(topics=['/ti_mmwave/micro_doppler']):
                    msg_handle = msg.message
                    time_domain_bins = msg_handle.time_domain_bins
                    nd = msg_handle.num_chirps
                    break
                xdata = np.empty((0,time_domain_bins*nd), float)
                ydata = []
                for msg in bag.read_messages(topics=['/ti_mmwave/micro_doppler']):
                    msg_handle = msg.message
                    mds_array = np.array(msg_handle.micro_doppler_array).reshape((nd, time_domain_bins))
                    # mds_array[int(nd/2),] = 0
                    xdata = np.append(xdata, mds_array.reshape((1,-1)), axis = 0)
                    ydata = np.append(ydata, label)
                # print(xdata.shape,'\n',ydata)
                # np.savetxt(csv_name_x, xdata, delimiter=",")
                # np.savetxt(csv_name_y, ydata, delimiter=",")
                np.save(csv_name_x, xdata)
                np.save(csv_name_y, ydata)
        print('*************************************************')
        print('Done. Total files : \n' + str(filecnt))
    def main(self):
        self.save_to_csv()

if __name__ == '__main__':
    micro_doppler_signature_proc().save_to_csv()