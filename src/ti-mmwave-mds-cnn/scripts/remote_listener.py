#!/usr/bin/env python
import rospy
from mds_cnn_node_pkg.msg import MDSPred
# from mds_dict import mds_dict

def callback(pred):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.prediction)
    print pred.prediction
   
def main():

   rospy.init_node('remote_listener')

   rospy.Subscriber('/ti_mmwave/mds_pred', MDSPred, callback)

   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':
   main()