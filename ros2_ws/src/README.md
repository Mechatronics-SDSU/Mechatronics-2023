### What is all this stuff ###

So this ROS workspace basically has the entry point for everything to work on the entire robot. The nodes have functionality implemented in outside classes but you need to run the nodes to get any of the components to get work together and do anything other than testing the classes.

By running the launch file, you can run every sensor for the robot with one command. Make sure you're in ros2_ws/src and then just 

    ros2 launch launch.py

And it's that easy, you have every sensor working for the robot ;)

If you need more in-depth data, read the topics.txt and it has some stuff in there. The nodes are mostly to interface with sensor data such as the DVL(velocity), ms5837(depth), ahrs(orientation) and then there's the zed node(position, vision, other stuff, everything basically lol). I have the zed node integrate with my own launch file so that's why there are so many topics. You can run the launch file and then run 

    ros2 topics list

to be able to see all the topics. Then you can subscribe to one using 

    ros2 topics echo /topic

And you'll see a bunch of data getting posted. For dres packages you need the can2ros_driver to be built first before you see anything useful (Connor or Joseph for more info)

In the PID package I put some pretty decent READme's on how it works. Pretty much all the sensor data gets fed to the PID for control. If you need to interface with any of the sensors just subscribe to there topics listed in topics.txt. I need to have the PID publish its ctrl_vals I'm still working on that.

## Some Help With Sourcing ##

When you clone the workspace, it won't come with the build or install folders, so you'll have to do that yourself. If you do colcon build, it'll give you some kind of ament_package error or something like that, the reason is that you need to source manually source /opt/ros/foxy/setup.bash After that, it will make an install folder for you, so you can just source that. 

Other questions @Zix on Discord (Conner S)
