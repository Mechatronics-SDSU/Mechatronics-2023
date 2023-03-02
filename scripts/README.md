## Directory for Scripts ##

&nbsp; &nbsp; Any code for automation of tasks will be placed here.

To run through a motor/sensor test and setup the robot for pool testing use the command 

    ssh 192.168.1.2 -X 
    
to ssh into the IP address we use as the default for the robot. Then run the command

    dbus-launch gnome-terminal
    
This will make a remote terminal able to run the startup script

    ./startup_script
    
