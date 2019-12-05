package org.firstinspires.ftc.robotlib.drivetrain;

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
