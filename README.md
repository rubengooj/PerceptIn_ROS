# PerceptIn_ROS (forked from Shuailing-Li version) #

We have slightly adapted the following repository from Shuailing-Li:

[https://github.com/Shuailing-Li/PerceptIn_ROS](https://github.com/Shuailing-Li/PerceptIn_ROS) 

to work in combination with our VO and SLAM algorithms with the stereo PerceptIn sensor and our ROS wrapper. 

Please notice that we have experienced some issues with the sensor and ROS, specially when initializing it, but in general it works nicely. 


### Usage:

To stream the output from the PerceptIn sensor to ROS, you need to start the roscore and open two different terminals executing the two following executables obtained after compiling the package in your ROS workspace:

./perceptin_server

./perceptin_client <exposure>  (default exposure: 600)

