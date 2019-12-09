package org.firstinspires.ftc.robotlib.state;

// The four main directions simulate joystick inputs to return the angle the robot would drive at in teleop
public enum AutoDirection
{
    FRONT(0),
    LEFT(270),
    RIGHT(90),
    REAR(180);

    private double angle;

    public double getAngle() { return angle; }

    AutoDirection(double angle) { this.angle = angle; }
}
