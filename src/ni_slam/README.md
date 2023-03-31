# Non-Iterative SLAM
  Non-iterative RGB-D-inertial SLAM
 
# Platform
  Codes have been tested on Ubuntu 16.04 with ROS kinetic.
  
  Currently, it doesn't support Ubuntu 14.04.

# Install Dependencies:
  1. Install FFT library: 
	```
	sudo apt-get install libfftw3-dev libfftw3-doc
	```
  2. Install trajectory viewer from hector_slam: 
 	```
  	sudo apt-get install ros-kinetic-hector-trajectory-server
  	```

# Install Intel Math Kernel Library (MKL):

  1. Download MKL from Intel website
  2. Extract downloaded file 
  	```
  	tar -zxvf [name_of_downloaded_file]
  	```
  3. Go to extracted folder, give permission: 
  	```
  	sudo chmod +x install.sh
  	```
  4. Run installation 
	```
  	./install.sh
  	```
  5. Link library, add to .bashrc: 
  	```
  	source /opt/intel/bin/compilervars.sh intel64
  	```
  6. Try compile in ROS workspace
 
 # Papers associated with this work
[Chen Wang](http://wangchen.online), [Junsong Yuan](http://www.ntu.edu.sg/home/jsyuan/), and [Lihua Xie](http://www.ntu.edu.sg/home/elhxie/), "Non-Iterative SLAM", The 18th International Conference on Advanced Robotics (ICAR 2017). (Best Paper Award) ([PDF available here](https://arxiv.org/pdf/1701.05294.pdf)) ([Video available here](https://www.youtube.com/watch?v=Ed_6wYIKRfs&feature=youtu.be))

[Chen Wang](http://wangchen.online), Minh-Chung Hoang, [Lihua Xie](http://www.ntu.edu.sg/home/elhxie/), [Junsong Yuan](http://www.ntu.edu.sg/home/jsyuan/), Non-iterative RGB-D-inertial Odometry, arXiv preprint arXiv:1710.05502, 2017 ([PDF available here](https://arxiv.org/pdf/1710.05502.pdf))
