# mirte_efficient_capsnet

This repository will contain code related to my thesis. Currently the goal of the thesis is to track objects' positions in a video, and use that as a world model. This will be tested in real life by implementing it on the mirte master, a robot developed by TU Delft.

I will build off Efficient-CapsNet, which seems to leverage the speed of attention with Capsule Networks' ability to recognize object by parts and position. Especially when using the CapsNet version proposed in Hinton's 2018 paper "MATRIX CAPSULES WITH EM ROUTING", the position of objects are build into the network.

Currently the plans for this repo are as follows:

1. Adept code to run as a ROS1 node. This allows for testing in real life during development
2. Adept code to use less memory. Current implementation cannot run on my Laptop due to the size of smallNORB dataset (after preprocessing)
3. Implement Matrix Capsules (excluding the EM routing - assuming attention works faster. Optional: implement EM routing as well to compare). The Matrix Capsules are supposed to hold objects' poses.
4. Change the architecture so that the network works on sequential data. I am expecting that attention, combined with the knowledge of previous time steps, should result in a network that is able to track objects using minimal computation.

## To run the ROS1 nodes:

### image_publisher
The image publisher can be used to publish pre-recorded images or those from a dataset. By default, the node will publish MNIST images at a rate of 1 image per second, and it will print the corresponding label. For other images you will have to change the code. To run the node, perform the following steps:

``` 
mamba activate ros_env  # Your environment may differ, just make sure to activate the correct environment
source devel/setup.bash  # Run this in the top level of your workspace
roscore

In a new terminal:
mamba activate ros_env  # Your environment may differ, just make sure to activate the correct environment
source devel/setup.bash  # Run this in the top level of your workspace
rosrun mirte_efficient_capsnet image_publisher.py 
```

You should now see something along these lines:

``` 
[INFO] [1725864223.522505]: Published an MNIST image with label: 7
[INFO] [1725864225.522887]: Published an MNIST image with label: 2
[INFO] [1725864227.523300]: Published an MNIST image with label: 1 
```