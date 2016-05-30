# stereocalib
OpenCV utility started as an exercise for calibrating a generic camera pair. Used for calibrating also RGB-D cameras. Stores calibration data in YAML format

# Example of RGBD camera calibration
python calibrate.py ../depthview/color/*.png --save f200_color.yaml
python calibrate.py ../depthview/ir/*.png --save f200_ir.yaml
python stereocalibrate.py --images1 ../depthview/color --images2 ../depthview/ir --calib1 f200_color.yaml --calib2 f200_ir.yaml --save f200.yaml

# Devices 
f200.yaml is for Real Sense F200 with serial 040140087709


#See Also
ROS based calibrator: http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

Visual Inertial Calibrator vis-calib: https://github.com/arpg/vicalib


The concept is trivial but some discussion for Kinect is in these two papers: 
	http://www.mas.bg.ac.rs/_media/istrazivanje/fme/vol43/1/8_bkaran.pdf
	http://www.cse.oulu.fi/~dherrera/papers/2012-KinectJournal.pdf

Related: https://github.com/erget/StereoVision/tree/master/stereovision