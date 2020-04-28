#!/usr/bin/env python
# Author: Feng Jin

import argparse
import rospy
import rosbag
import numpy as np
import math
import subprocess
import matplotlib.pyplot as plt
from micro_doppler_pkg.msg import MicroDoppler
from ti_mmwave_rospkg.msg import RadarScan
import os

class falling_proc:
    def __init__(self):
        self.bagsrcdir  = '/home/ece561/rosbag/SevaTraining/' 
        self.csvdir     = '/home/ece561/rosbag/SevaTraining/'
        self.shiftnum   = 0     

    def filter(self):
        for file in os.listdir(self.bagsrcdir):
            if file.endswith(".bag") & file.startswith("sitting"):
                bag = rosbag.Bag(self.bagsrcdir + file)
                stamp = np.array([''])
                for msg in bag.read_messages(topics=['/ti_mmwave/micro_doppler']):
                    time = str(msg.message.header.stamp.secs) + '.' + str(msg.message.header.stamp.nsecs)
                    stamp = np.append(stamp, np.array([time]))
                stamp = stamp[1:]
                timefile = file[:-4] + '.csv'
                falling_time = np.genfromtxt(self.csvdir + timefile,delimiter=',')
                num_falling = np.size(falling_time, 0)
                falling_filter_arg = '\"('
                for i in range(num_falling):
                    falling_filter_arg = falling_filter_arg + '((t.secs>=' + stamp[int(falling_time[i,0])-self.shiftnum] + ')&(' + 't.secs<=' + stamp[int(falling_time[i,1])+self.shiftnum] +'))or'
                falling_filter_arg = falling_filter_arg[:-2] + ')\"'

                nothing_filter_arg = '\"('
                for i in range(num_falling-1):
                    nothing_filter_arg = nothing_filter_arg + '((t.secs>=' + stamp[int(falling_time[i,1])+self.shiftnum] + ')&(' + 't.secs<=' + stamp[int(falling_time[i+1,0])-self.shiftnum] +'))or'
                nothing_filter_arg = nothing_filter_arg[:-2] + ')\"'

                print('********************************')
                # print(falling_filter_arg)
                unfiltered_file = self.bagsrcdir + file
                filtered_file = self.bagsrcdir + "processed/" + file
                command = "rosbag filter " + unfiltered_file + ' ' + filtered_file + ' ' + falling_filter_arg
                # print(command)

                # unfiltered_file = self.bagsrcdir + file
                # filtered_file = self.bagsrcdir + "processed/" + 'nth_' + file[:-5] + '0.bag'
                # self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)
                # command = "rosbag filter " + unfiltered_file + ' ' + filtered_file + ' ' + nothing_filter_arg
                
                self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True)
                # print(command)

                while 1:
                    poll = self.p.poll()
                    if(poll!=None):
                        # kill the child process
                        killcommand = "kill -9 " + str(self.p.pid)
                        self.k = subprocess.Popen(killcommand, shell=True)
                        break
                # print(timefile)
                # print("**********************")
    def main(self):
        self.filter()

if __name__ == '__main__':
    falling_proc().filter()