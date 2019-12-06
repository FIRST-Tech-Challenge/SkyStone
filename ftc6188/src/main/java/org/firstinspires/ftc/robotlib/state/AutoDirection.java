package org.firstinspires.ftc.robotlib.state;

// The four main directions simulate joystick inputs to return the angle the robot would drive at in teleop
public enum AutoDirection
{
    FRONT(Math.atan2(1, 0) - Math.PI/2),
    LEFT(Math.atan2(0, -1) - Math.PI/2),
    RIGHT(Math.atan2(0, 1) - Math.PI/2),
    REAR(Math.atan2(1, 0) - Math.PI/2);

    private double angle;

    public double getAngle() { return angle; }

    AutoDirection(double angle) { this.angle = angle; }
}
