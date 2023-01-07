# Subsystems

### Drive Subsystem
This subsystem is in charge of controlling the robots general mobility. The core funtionality includes teleoperated driving modes and interfacing with general purpose position or orientation sensors connected to the motor controllers. Other autonomous functions to control robot positioning are also implemented here. Here are the functions defined at the subsystem level:
- Periodic: update the filter with the current velocity value of the left speed controller
- CurvatureDrive: drive via a forward velocity and radius of curvature
- ArcadeDrive: drive via a forward velocity and rotational velocity
- ResetEncoder: this will set the encoders position to 0. Useful for mitigating accumulated error
- DriveToDistance: this function will drive the robot a specific distance along a line (no ability to change orientation)
- GetVelocity: this will return the left speed controllers (filtered!) velocity. Primarily for testing purposes
- GetWheelSpeeds: this will return a set of left and right wheel speeds based on a linear and angular velocity of the differential drive
