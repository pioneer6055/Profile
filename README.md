# Profile
Library to store a set of movement commands and execute the commands to drive the robot.

This is a C++ library to provide autonomous robot movements in a simple format.  You don't
need to use a path planner or load any files at runtime.  You can input movement commands
directly in your code and then execute them.  Profile uses a trapezoidal motion profile
based on distance (or rotational) feedback to execute movements.  You should use the 
average of both sides encoders if using a skid-steer robot to help account for distance 
variances due to wheel scrubbing during turns.  While not as accurate as a pre-planned 
path profiler, it is easier to implement and understand.

Currently implemented actions are:
MOVE  (drive in a straight line forward or backwards for a certain distance)
TURN  (turn to a new heading)
PAUSE (pause for a period of milliseconds)
CURVE (drive in a curved line for a certain distance)

Requirements:  
Heading value from a gyro and distance value from an encoder
