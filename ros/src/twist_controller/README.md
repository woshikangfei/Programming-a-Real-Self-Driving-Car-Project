# DBW Node Overview
Once messages are being published to /final_waypoints, the vehicle's waypoint follower will publish twist commands to the /twist_cmd topic. The goal for this part of the project is to implement the drive-by-wire node (dbw_node.py) which will subscribe to /twist_cmd and use various controllers to provide appropriate throttle, brake, and steering commands. These commands can then be published to the following topics:

- /vehicle/throttle_cmd
- /vehicle/brake_cmd
- /vehicle/steering_cmd


Since a safety driver may take control of the car during testing, you should not assume that the car is always following your commands. If a safety driver does take over, your PID controller will mistakenly accumulate error, so you will need to be mindful of DBW status. The DBW status can be found by subscribing to /vehicle/dbw_enabled.

When operating the simulator please check DBW status and ensure that it is in the desired state. DBW can be toggled by clicking "Manual" in the simulator GUI.

All code necessary to implement the drive-by-wire node can be found in the package:

ros/src/twist_controller

Twist controller package files

Within the twist controller package, you will find the following:

### dbw_node.py
This python file implements the dbw_node publishers and subscribers. You will need to write ROS subscribers for the /current_velocity, /twist_cmd, and /vehicle/dbw_enabled topics. This file also imports the Controller class from twist_controller.py which will be used for implementing the necessary controllers. The function used to publish throttle, brake, and steering is publish.

Note that throttle values passed to publish should be in the range 0 to 1, although a throttle of 1 means the vehicle throttle will be fully engaged. Brake values passed to publish should be in units of torque (N*m). The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.

### twist_controller.py
This file contains a stub of the Controller class. You can use this class to implement vehicle control. For example, the control method can take twist data as input and return throttle, brake, and steering values. Within this class, you can import and use the provided pid.py and lowpass.py if needed for acceleration, and yaw_controller.py for steering. Note that it is not required for you to use these, and you are free to write and import other controllers.

### yaw_controller.py
A controller that can be used to convert target linear and angular velocity to steering commands.

### pid.py
A generic PID controller that can be used in twist_controller.py.

### lowpass.py
A generic low pass filter that can be used in lowpass.py.