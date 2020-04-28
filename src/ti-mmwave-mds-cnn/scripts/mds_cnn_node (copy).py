#!/usr/bin/env python
# Author: Leo Zhang

import rospy
import numpy as np
import tensorflow as tf
from keras.models import model_from_json
from keras.models import load_model
import os
import math
from micro_doppler_pkg.msg import MicroDoppler_m
from mds_cnn_node_pkg.msg import MDSPred
from mds_dict import mds_dict
import socket


class mds_cnn_node:
    def __init__(self):
        self.frame_id = 'mds_pred'
        self.directory = './output/'
        
        self.loaded_model = load_model(self.directory+'mds_cnn_model.h5')
        self.loaded_model._make_predict_function()
        self.graph = tf.get_default_graph()

        self.nd = 128
        self.time_domain_bins = 20
        print('Loaded model from disk.')

    def main(self):
        self.serversocket  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        rospy.init_node('mds_cnn_node')
        self.sub_ = rospy.Subscriber('/ti_mmwave/micro_doppler', MicroDoppler_m, self.mds_predictor)
        self.pub_ = rospy.Publisher('/ti_mmwave/mds_pred', MDSPred, queue_size=100)
        rospy.spin()

    def pred_state(self):
        state = 'Nothing'
        arr = self.pred.flatten()
        tmp = np.argmax(arr)
        if arr[tmp] > .8:
            state = str(mds_dict[tmp])
        return state

    def mds_predictor(self, mds):
        tmp = np.array(mds.micro_doppler_array)
        if max(tmp) != 0:
            tmp = tmp / max(tmp)
        test_data = tmp.reshape(-1, self.nd, self.time_domain_bins, 1)

        with self.graph.as_default():
            self.pred = self.loaded_model.predict(test_data)
            
        cur_state = self.pred_state()
        mds_pred_msg = MDSPred()
        mds_pred_msg.header.frame_id = self.frame_id
        mds_pred_msg.header.stamp = rospy.Time.now()
        mds_pred_msg.mds_pred_array = self.pred.flatten().tolist()
        mds_pred_msg.prediction = cur_state
        mds_pred_msg.target_idx = mds.target_idx
        mds_pred_msg.posX = mds.posX
        mds_pred_msg.posY = mds.posY
        mds_pred_msg.velX = mds.velX
        mds_pred_msg.velY = mds.velY
        self.pub_.publish(mds_pred_msg)

  	dst_ip = "150.135.222.168"
        dst_port = 5000
        message =  'This is patient behavior detection info.' + \
                    ' Target Idx:' + str(mds.target_idx) + \
                    ' PosX:' + str(round(mds.posX,1)) + \
                    ' PosY:' + str(round(mds.posY,1)) + \
                    ' VelX:' + str(round(mds.velX,1)) + \
                    ' VelY:' + str(round(mds.velY,1)) + \
                    ' Behavior:' + cur_state  + \
                    ' END!'       
        self.serversocket.sendto(message.encode(), (dst_ip, dst_port))

if __name__ == '__main__':
    mds_cnn_node().main()
