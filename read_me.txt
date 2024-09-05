To launch this controller with the turtlebot3 simulation:
ros2 launch excavator_control turtle.launch.py

Lookahead distance, the control gains, and any other parameters are
currently all set in the diff_drive_pure_pursuit_updated_copy.py file
but I will work on setting up a YAML file in the future

To disable E_STOP and publish twist messages:
ros2 topic pub /STOP std_msgs/msg/Bool 'data: False' 

TO USE WITH THE PHYSICAL EXCAVATOR:
1.Go to /excavator_control/config/params.YAML
2.Ensure the following parameters are set:
    a. odom_topic: 'GPSfix'
    b. command_vel_topic: 'pid_twist'
    c. simulate_turtlebot: False

**Currently, the portions of the launch file that launch gazebo, the markers, and rviz
are commented out, they should be uncommented for use with simulation