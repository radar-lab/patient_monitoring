## Project Title:
[Multiple Patients Behavior Detection in Real-time using mmWave Radar and Deep CNNs](https://ieeexplore.ieee.org/abstract/document/8835656)

## Project Discription
To address potential gaps noted in patient monitoring in the hospital, a novel patient behavior detection system using mmWave radar and deep convolution neural network (CNN), which supports the simultaneous recognition of multiple patients’ behaviors in real-time, is proposed. In this study, we use an mmWave radar to track multiple patients and detect the scattering point cloud of each one. For each patient, the Doppler pattern of the point cloud over a time period is collected as the behavior signature. A three-layer CNN model is created to classify the behavior for each patient. The tracking and point clouds detection algorithm was also implemented on an mmWave radar hardware platform with an embedded graphics processing unit (GPU) board to collect Doppler pattern and run the CNN model. A training dataset of six types of behavior were collected, over a long duration, to train the model using Adam optimizer with an objective to minimize cross-entropy loss function. Lastly, the system was tested for real-time operation and obtained a very good inference accuracy when predicting each patient’s behavior in a two-patient scenario.

## Project Source Code
Run 'permission.sh' to give the serial port permission. 

Run 'behavior.sh' to run the ROS nodes sequentially. 

Check ~/src folder for the project source code. 
 
## System Architecture
![](/misc/system.JPG)

## Micro-Doppler Pattern for Different Motions
![](/misc/microDoppler.JPG)

## Experiment Setup
![](/misc/experiment.jpg)

## Motion Detection in Real-Time
![](/misc/results.gif)
