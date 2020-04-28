#!/usr/bin/env python
# Author: Leo Zhang

import argparse
import rospy
import rosbag
import numpy as np
import math
import matplotlib.pyplot as plt
from micro_doppler_pkg.msg import MicroDoppler
from ti_mmwave_rospkg.msg import RadarScan

class micro_doppler_signature_proc:
    def __init__(self, bag, label):
        self.bag = rosbag.Bag('/home/ece561/rosbag/' + bag)
        self.csv_name_x = '/home/ece561/database/x_' + bag[:-3] + 'csv'
        self.csv_name_y = '/home/ece561/database/y_' + bag[:-3] + 'csv'
        self.label = label
        
    def read_plot_mds_all(self):
        plt.figure(figsize=(20, 20))
        for msg in self.bag.read_messages(topics=['/ti_mmwave/micro_doppler']):
            msg_handle = msg.message
            time_domain_bins = msg_handle.time_domain_bins
            nd = msg_handle.num_chirps
            break

        mds_show = np.empty((nd,0), float)
        for msg in self.bag.read_messages(topics=['/ti_mmwave/micro_doppler']):
            msg_handle = msg.message
            mds_array = np.array(msg_handle.micro_doppler_array).reshape((nd, time_domain_bins))
            # mds_array[int(nd/2),] = 0
            mds_show = np.append(mds_show, np.vstack(mds_array[:,-1]), axis = 1)
        # plt.xlim(1.3, 4.0)
        # plt.figure(figsize=(10,6))
        # print(mds_show.shape)
        plt.imshow(mds_show)
        plt.colorbar()
        plt.show()
    
    def read_plot_mds_array(self):
        plt.figure(figsize=(20, 20))
        for msg in self.bag.read_messages(topics=['/ti_mmwave/micro_doppler']):
            msg_handle = msg.message
            time_domain_bins = msg_handle.time_domain_bins
            nd = msg_handle.num_chirps
            mds_array = np.array(msg_handle.micro_doppler_array).reshape((nd, time_domain_bins))
            # mds_array[int(nd/2),] = 0
            plt.imshow(mds_array)
            print(str(msg_handle.header.stamp.secs) + '.' + str(msg_handle.header.stamp.nsecs))
            # plt.pause(.00001)
            if plt.waitforbuttonpress(.00001) == True:
                while plt.waitforbuttonpress() :
                    break

    def save_to_csv(self):
        for msg in self.bag.read_messages(topics=['/ti_mmwave/micro_doppler']):
            msg_handle = msg.message
            time_domain_bins = msg_handle.time_domain_bins
            nd = msg_handle.num_chirps
            break
        self.x = np.empty((0,time_domain_bins*nd), float)
        self.y = []
        for msg in self.bag.read_messages(topics=['/ti_mmwave/micro_doppler']):
            msg_handle = msg.message
            mds_array = np.array(msg_handle.micro_doppler_array).reshape((nd, time_domain_bins))
            # mds_array[int(nd/2),] = 0
            self.x = np.append(self.x, mds_array.reshape((1,-1)), axis = 0)
            self.y = np.append(self.y, self.label)
        print(self.x.shape,'\n',self.y)
        np.savetxt(self.csv_name_x, self.x, delimiter=",")
        np.savetxt(self.csv_name_y, self.y, delimiter=",")
        
    def main(self):
        self.save_to_csv()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='bag & label')
    parser.add_argument(
        'bag',
        type=str,
        nargs='?',
        default='2018-06-01-19-28-05.bag',
        help='Bag name.'
    )
    parser.add_argument(
        'label',
        type=int,
        nargs='?',
        default=0,
        help='label name'
    )
    args = parser.parse_args()
    bag = vars(args)['bag']
    label = vars(args)['label']
    micro_doppler_signature_proc(bag, label).read_plot_mds_all()
    # micro_doppler_signature_proc(bag, label).read_plot_mds_array()
    # micro_doppler_signature_proc(bag, label).save_to_csv()
    # micro_doppler_signature_proc(bag, label).main()