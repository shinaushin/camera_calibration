Conducts camera calibration based on 20 images taken by iPhone 6s+ of asymmetric circle grid.

20 images are stored in data folder. 
There are two nodes: send_images, calibrate

Program can be launched using: roslaunch camera_calibration start_calibration.launch.
Service callback activates calibration process upon receipt of Calibrate.srv message.
The message's request is a simple string indicating the mode of the program.
The service can be called from the terminal using: rosservice call /calibrate "calibrate".

Example output YAML file can be found in root of package folder: intrinsics.yml

Final RMS: 0.205215
