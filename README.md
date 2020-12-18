# akros_drive

### Description
ROS node to drive the AKROS Differential (Jetbot2) and Ackermann (Jetracer2/ROSCar) platforms at a particular linear and angular velocity. The following Python scripts are provided:
* drive_ackermann.py
* drive_differential.py
This node requires the Adafruit MotorHat libraries (Differential) or the ros-i2cpwmboard package (Ackermann) to be installed for the individual platforms

### Configuration
Configuration files are located in the 'config' directory and contain the parameters for each platform. The following configuration files are provided:
* config_ackermann.py : Calibrated center values for steering and throttle
* config_differential.py : Maximum PWM and calibrated gain values for linear and angular velocities

### Subscribers
Both scripts subscribe to the `/cmd_vel` topic, compute the individual PWM values for each motor and send them via i2c to the individual motor controllers (Adafruit MotorHat or PCA9685 Servo Controller)

### Publishers
The Differential script does not publish any topic, although it can be modified to publish the PWM values sent to the motor driver. The Ackermann script publishes the servo values as an array of type 'ServoArray' from the ros-i2cpwmboard package. 

### Launch
For each script, a launch file is provided:
* drive_ackermann.launch: `roslaunch akros_drive drive_ackermann.launch`
* drive_differential.launch `roslaunch akros_drive drive_differential.launch`
