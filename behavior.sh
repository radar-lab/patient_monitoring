#!/bin/sh

xterm -title "Radar Module" -e "source ~/.bashrc; roslaunch ti_mmwave_rospkg 1642_short_range.launch"&

sleep 5 

xterm -title "Microdoppler" -e "source ~/.bashrc; cd ti_ros/src/micro_doppler_pkg/scripts/; python micro_doppler_m.py"&

sleep 5 

xterm -title "CNN Node" -e "source ~/.bashrc; cd ti_ros/src/ti-mmwave-mds-cnn/scripts/; python3 mds_cnn_node.py"&

sleep 20 

xterm -title "Seva Aggregator" -e "source ~/.bashrc; cd ti_ros/src/dpa-mmwave-temp-integration/src-ros/; python seva_event_aggregator_module.py"&

xterm -title "Seva Event Executor" -e "source ~/.bashrc; cd ti_ros/src/dpa-mmwave-temp-integration/src-ros/; python seva_event_executor.py"&
