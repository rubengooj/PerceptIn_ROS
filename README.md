# PerceptIn_ROS (forked from Shuailing-Li version) #

We have slightly adapted the following repository from Shuailing-Li:

[https://github.com/Shuailing-Li/PerceptIn_ROS](https://github.com/Shuailing-Li/PerceptIn_ROS) 

to work in combination with our VO and SLAM algorithms with the stereo PerceptIn sensor and our ROS wrapper. 

Please notice that we have experienced some issues with the sensor and ROS, specially when initializing it, but in general it works nicely.


## Usage 
To stream the output from the PerceptIn sensor to ROS, you can follow one of the next alternatives:

### roslaunch:
There is an example launch file in perceptin_pynodes/launch. The exposition parameter is contained in the file:

roslaunch perceptin_pynodes PerceptIn.launch

### rosrun:
We developed the perceptin_pynodes package to avoid the execution of the binaries. You need to start a **roscore** and then open two terminals and run:

rosrun perceptin_pynodes Perceptin_Server_node
rosrun perceptin_pynodes Perceptin_Client_node

>Before running the client, you can set the exposition by tiping:
>rosparam set exposition <exposure>  (default exposure: 600)

### executables:

You need to start a **roscore** and open two different terminals executing the two following executables obtained after compiling the package in your ROS workspace (build):

./perceptin_server

./perceptin_client <exposure>  (default exposure: 600)

