robot

Thee robot folder contains all the robots used in teleop.  A Robot in this sense is a class containing references to all the motors servos sensors and anything else
on the robot as well as their initial states.  It also contains the Robots drivetrain if applicable or other types of servos and motors.  Each teleop code must
have a robot as without it all the initialization must take place in the teleop code.