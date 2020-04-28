import matplotlib.pyplot as plt
import rospy
import rosbag
import numpy as np
import argparse

cmd_parser  = argparse.ArgumentParser()
cmd_parser.add_argument('--filename', type=str, default = None)
args        = cmd_parser.parse_args()

bag         = rosbag.Bag(args.filename)
mds_array   = []
for msg in bag.read_messages(topics=['/ti_mmwave/micro_doppler']):
    msg_handle          = msg.message
    nd                  = msg_handle.num_chirps
    time_domain_bins    = msg_handle.time_domain_bins
    mds_data            = np.array(msg_handle.micro_doppler_array).reshape((nd, time_domain_bins))
    mds_array.append(mds_data[:, 1])

mds_array = np.array(mds_array)
mds_array = np.transpose(mds_array)
print(mds_array.shape)
plt.imshow(mds_array)
plt.show()