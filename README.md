# tottori_cvlab
computer vision in tottori-cvlab
1. calibarate IR-Color cameras with no-overlapping cameras
 (1)calibrate laser on the board:calibrate_laser_version3
 (2)calibrate internal parameters of IR-RGB camera(e.g. kinect):internal calibrate(IR and RGB)
 (3)calibrate depth parameters of depth camera:depth_calibration(IR_D)_ver2
 (4)calibrate relative pose(R,T) of IR and RGB cameras inside one Kinect:stereo calibration(IR_RGB)
 (5)collecte data for calibrating relative pose(R,T) between two non-overlapping kinect:calibrate_laser_version3
 (6)calibrate relative pose (R,T) between two non-overlapping kinect based on collected data: calibrate_based on stored data1
 <calibrate 2kinects_version1> join step-5 and step-6 togethor
