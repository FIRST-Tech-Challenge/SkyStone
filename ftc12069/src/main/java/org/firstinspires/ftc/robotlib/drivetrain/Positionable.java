package org.firstinspires.ftc.robotlib.drivetrain;

public interface Positionable
{
    void setTargetPosition(double position);
    double getCurrentPosition();
    double getTargetPosition();
    void updatePosition();
    boolean isPositioning();
    void finishPositioning();
    void position();
    double getTicksPerUnit();
}
