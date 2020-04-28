#!/usr/bin/env python
# Author: Leo Zhang

import argparse
import rospy
import numpy as np
import math
from micro_doppler_pkg.msg import MicroDoppler_m
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

        self.mds_array = np.zeros((self.nd, self.time_domain_bins, 253))
        self.prev_id = 0
        self.tmp_data = np.empty((8,0))

    def ti_doppler_parser(self, radar):
        if radar.point_id < self.prev_id:
            self.micro_doppler()
        self.prev_id = radar.point_id
        if radar.target_idx < 253:
            tmp = np.array([radar.target_idx, radar.range, radar.doppler_bin, radar.intensity, radar.posX, radar.posY, radar.velX, radar.velY])
            self.tmp_data = np.append(self.tmp_data, tmp.reshape((-1,1)), axis = 1)
        

    def micro_doppler(self):
        targets = np.array(np.int_(np.unique(self.tmp_data[0,:])))
        targets_sorting = np.zeros((targets.size,4))
        mds_cur = np.zeros((targets.size,self.nd))
        for i in range(self.tmp_data.shape[1]):
            tmp = np.where(targets == int(self.tmp_data[0,i]))
            # range compensation
            mds_cur[tmp[0][0],int(self.tmp_data[2,i])] += self.tmp_data[3,i] * self.tmp_data[1,i]**2
            targets_sorting[tmp[0][0],0] = self.tmp_data[4,i]
            targets_sorting[tmp[0][0],1] = self.tmp_data[5,i]
            targets_sorting[tmp[0][0],2] = self.tmp_data[6,i]
            targets_sorting[tmp[0][0],3] = self.tmp_data[7,i]

        if targets.size > 1:
            for i in range(targets.size):
                tmp = mds_cur[i,:].flatten()
                # normalize every micro-doppler signature
                temp = max(tmp)
                if temp != 0:
                    mds_norm = tmp / temp
                else:
                    mds_norm = tmp
                temp1 = np.delete(self.mds_array[:,:,targets[i]], 0, 1)
                temp2 = np.transpose([mds_norm])
                self.mds_array[:,:,targets[i]] = np.append(temp1, temp2, axis=1)
                mds_list = self.mds_array[:,:,targets[i]].flatten().tolist()
                
                # publish msg
                mds_msg = MicroDoppler_m()
                mds_msg.header.frame_id = self.frame_id
                mds_msg.header.stamp = rospy.Time.now()
                mds_msg.time_domain_bins = self.time_domain_bins
                mds_msg.num_chirps = self.nd
                mds_msg.target_idx = targets[i]
                mds_msg.micro_doppler_array = mds_list
                mds_msg.posX = targets_sorting[i,0]
                mds_msg.posY = targets_sorting[i,1]
                mds_msg.velX = targets_sorting[i,2]
                mds_msg.velY = targets_sorting[i,3]
                
                self.pub_.publish(mds_msg)
        elif targets.size == 1:
            tmp = mds_cur[0,:].flatten()
            # normalize every micro-doppler signature
            temp = max(tmp)
            if temp != 0:
                mds_norm = tmp / temp
            else:
                mds_norm = tmp
            temp1 = np.delete(self.mds_array[:,:,targets], 0, 1)
            temp2 = np.transpose([mds_norm])
            self.mds_array[:,:,targets] = np.append(temp1, temp2, axis=1)
            mds_list = self.mds_array[:,:,targets].flatten().tolist()
            
            # publish msg
            mds_msg = MicroDoppler_m()
            mds_msg.header.frame_id = self.frame_id
            mds_msg.header.stamp = rospy.Time.now()
            mds_msg.time_domain_bins = self.time_domain_bins
            mds_msg.num_chirps = self.nd
            mds_msg.target_idx = targets
            mds_msg.micro_doppler_array = mds_list
            mds_msg.posX = targets_sorting[0,0]
            mds_msg.posY = targets_sorting[0,1]
            mds_msg.velX = targets_sorting[0,2]
            mds_msg.velY = targets_sorting[0,3]
            self.pub_.publish(mds_msg)

        self.tmp_data = np.empty((8,0))

    def main(self):
        rospy.init_node('micro_doppler_node')
        self.sub_ = rospy.Subscriber('/ti_mmwave/radar_scan', RadarScan, self.ti_doppler_parser)
        self.pub_ = rospy.Publisher('/ti_mmwave/micro_doppler', MicroDoppler_m, queue_size=100)
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