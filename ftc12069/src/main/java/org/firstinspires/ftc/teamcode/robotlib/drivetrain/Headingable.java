package org.firstinspires.ftc.teamcode.robotlib.drivetrain;

public interface Headingable extends Rotatable
{
    void setTargetHeading(double targetHeading);

    double getCurrentHeading();
    double getTargetHeading();

    void updateHeading();
    void rotate();

    boolean isRotating();
    void finishRotating();
}
