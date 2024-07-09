To launch this controller with the turtlebot3 simulation:
ros2 launch excavator_control turtle.launch.py

Lookahead distance, the control gains, and any other parameters are
currently all set in the diff_drive_pure_pursuit_updated_copy.py file
but I will work on setting up a YAML file in the future

To disable E_STOP and publish twist messages:
ros2 topic pub /STOP std_msgs/msg/Bool 'data: false' 
