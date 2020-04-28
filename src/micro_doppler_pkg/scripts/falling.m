clear all; close all; clc;

bag = rosbag('/home/ece561/rosbag/falling10_3.bag');
sel = select(bag,'Topic','/ti_mmwave/micro_doppler');
micro_doppler = readMessages(sel,'DataFormat','struct');
% micro_doppler = readMessages(sel);

nd = micro_doppler{1}.NumChirps;
time_domain_bins = micro_doppler{1}.TimeDomainBins;
len = size(micro_doppler, 1);
mds_show = zeros(nd, len);

for i = 1 : len
    mds_array = reshape(micro_doppler{i}.MicroDopplerArray, time_domain_bins, nd);
    mds_show(:, i) = mds_array(20,:);
end

imagesc(mds_show);
colorbar