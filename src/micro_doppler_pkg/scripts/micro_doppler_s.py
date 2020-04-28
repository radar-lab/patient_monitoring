#!/usr/bin/env python
# Author: Leo Zhang

import argparse
import rospy
import numpy as np
import math
from micro_doppler_pkg.msg import MicroDoppler
from ti_mmwave_rospkg.msg import RadarScan

class micro_doppler_signature:
    def __init__(self, nd = 0, host_velocity = 0):
        self.frame_id = 'micro_doppler'
        self.host_velocity = host_velocity
        self.time_domain_bins = 20

        if nd == 0:
            print("Waiting for parameters...")
            while not rospy.has_param("/ti_mmwave/numLoops"):
                continue
            self.nd = rospy.get_param("/ti_mmwave/numLoops")
            print("Parameters got. Continue to publish topics.")
        else:
            self.nd = nd

        self.mds_array = np.zeros((self.nd, self.time_domain_bins))
        self.prev_id = 0
        self.mds_cur = np.zeros(self.nd)  

    def ti_doppler_parser(self, radar):
        if radar.point_id < self.prev_id:
            self.micro_doppler()
        self.prev_id = radar.point_id
        # range compensation
        temp = radar.intensity * radar.range**2
        self.mds_cur[radar.doppler_bin] += temp

    def micro_doppler(self):
        # remove static clutter
        # self.mds_cur[int(self.nd/2)] = 0
        # normalize every micro-doppler signature
        temp = max(self.mds_cur)
        if temp != 0:
            mds_norm = self.mds_cur / temp
        else:
            mds_norm = self.mds_cur
        temp1 = np.delete(self.mds_array, 0, 1)
        temp2 = np.transpose([mds_norm])
        self.mds_array = np.append(temp1, temp2, axis=1)
        mds_list = self.mds_array.flatten().tolist()
        
        # publish msg
        mds_msg = MicroDoppler()
        mds_msg.header.frame_id = self.frame_id
        mds_msg.header.stamp = rospy.Time.now()
        mds_msg.time_domain_bins = self.time_domain_bins
        mds_msg.num_chirps = self.nd
        mds_msg.micro_doppler_array = mds_list
        self.pub_.publish(mds_msg)

        self.mds_cur = np.zeros(self.nd)

    def main(self):
        rospy.init_node('micro_doppler_node')
        self.sub_ = rospy.Subscriber('/ti_mmwave/radar_scan', RadarScan, self.ti_doppler_parser)
        self.pub_ = rospy.Publisher('/ti_mmwave/micro_doppler', MicroDoppler, queue_size=100)
        rospy.spin()
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='number of chirps')
    parser.add_argument(
        'num_chirps',
        type=int,
        nargs='?',
        default=0,
        help='Number of chirps.'
    )
    args = parser.parse_args()
    nd = vars(args)['num_chirps']
    micro_doppler_signature(nd, host_velocity=0).main()
    
    
