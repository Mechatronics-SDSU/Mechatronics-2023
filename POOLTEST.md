Pool Test Documentation

## STEP 1 ##
SSH into correct IP address, you’ll have to find this by running curl ifconfig.me on the robot orin. You may also have to do some network configuration (Ken will inevitably get sucked into this)

Example:
ssh -X -C 146.244.98.15
Note: don’t forget the -X and -C flags because we need them for GUI forwarding!! (VSCode)

## STEP 2 ##
When you’re SSHd in, you’ll have to run the tmux session. You’re in luck because there’s a script that does this so you just have to navigate to home/mechatronics/master/Mechatronics-2023/scripts

Then you’ll run ./tmux_script.sh
This will pull up a session with all the windows you need and they’ll all be sourced for ROS2. You’ll have a /src directory for the launch file, 3 /ros directories for running nodes, and a /scripts directory for scripts

## STEP 3 ##
Before turning on the robot we’ll do some quick motor and sensor tests on the robot. So run the motor test script first which will also enable CAN. Validate that all the motors are moving with no issues.

## STEP 4 ##
Run the sensor test script if applicable/available

## STEP 5 ##
Choose which nodes to run in the launch file, comment out the ones we don’t need. Launch the robot to test its functionality. To edit the robot functionality, you’ll have to edit the source code of the nodes and recompile them using colcon build –packages-select some_package

## STEP 6 ##
If we won’t be running all robot functionality you can run individual scripts and ros nodes as needed. To run a ROS node an example command is ros2 run controller_node controller_exec

## Next Pool Test ##
First Pool Test May 27th

Follow instructions up to Step 4
Start the picture taking script in vision/
Start the controller and joy_node ROS nodes
Make sure the robot can move with no leaks and take pictures for training.

