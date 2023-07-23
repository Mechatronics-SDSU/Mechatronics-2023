### ROS Layout ###

  To keep our nodes organized, we have to have a fairly strict structure on what to include in the ros2_ws

#### 3 Important Files/Directories To Know About ####

&nbsp; &nbsp; First, we have our scion_types directory. Here, we can store all the custom messages to be used on the robot in one place. For example, orientation data should be communicated as a float32[] array which isn't in the std_msgs package given by ROS. So in the scion_types msg directory we can define this custom type to easily pass around orientation data as an array. This directory has to be built first for all the nodes to work (since they're dependent on those custom types) If you add a new type, you HAVE to add it to the CMakeLists.txt or the compiler won't know it exists.

&nbsp; &nbsp; Second, we have the launch.py file. This is so we can spin all our nodes from one place. If you ever add a node to the workspace, after testing that it works, make sure to put your node in the launch file. Use the current nodes as a reference. You need to specify the package it came from and the executable name (try to keep these consisent system_node, system_exec). You can also give your nodes arguments based on how you want them to behave.

&nbsp; &nbsp; Third is the topics.txt file. This is a simple text file where we need to keep good documentation of the available topics and EXACTLY what information we can expect to get ESPECIALLY if you write in python since the data types aren't clear. This means that if your topics is an array of floats [float1, float2, float3], make sure to define exactly what we can expect float1, float2, and float3, to be. Keep topic names consistent to what is already in the file. Use three words ending with _data. General format will be system_type_data. The _data may seem redundant but if something in the code ends in _data, we know it's a topic (not an object, function, file, etc.). Even if we know AHRS gives orientation data, anybody should be able to look at the topic with no previous knowledge and be able to put together what it's for (I don't know about you geniuses but I still don't know what AHRS means)

#### Correctly Adding a Node ####

&nbsp; &nbsp; To make sure we're not poluting the workspace, the nodes should really just contain code specifically pertaining to ROS nodes with minimal functionality code. In other words, if you have something to implement, for example, orientation data, don't write the AHRS functionality and driver within the node. Instead, it should be its own class in the classes directory. In the ahrs_node file (all ROS nodes in lowercase by the way), simply import your class and instantiate the relevant object. Then call its functions for the functionality you need, and publish the data you get or make a request using the data or whatever you're gonna do. But all the code in the ROS Node should be 100% necessary for the ROS node functionality, not the class functionality. Also we should end all our node names with _node because it just keeps things clear and obvious. (For example, big difference between ahrs and ahrs_node)
    
###### ROS workspace questions ask Conner Sommerfield @Zix on Discord ######
