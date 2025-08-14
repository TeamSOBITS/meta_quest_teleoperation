# meta_quest_teleoperation

## Controls
- Right Primary action button (A): publishes "Starting demonstration" to the topic "/demonstration_indicator"
- Right Secondary action button (B): publishes "Stopping demonstration" to the topic "/demonstration_indicator"
- Right Trigger: toggle true/false and publish a boolean to the topic "/gripper_button"
- Right Grip: clutch in the teleoperation control. The controlled position only changes when clutched in. The controller position is published to both /right_controller_odom and /tf as an Odom and TF message, respectively. 
- Right Analog stick: left/right rotates the frame, allowing for aligning the unity frame with the robot frame
- Left Primary action button (A): Opens a keyboard to change the IP of the ROS PC. 
