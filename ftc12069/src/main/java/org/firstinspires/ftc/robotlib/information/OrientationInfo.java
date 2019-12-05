package org.firstinspires.ftc.robotlib.information;

/**
 * Stores an angle and rotation for orientating the robot
 */
public class OrientationInfo {
    public double angle;
    public double rotation;

    public OrientationInfo(double angle, double rotation) {
        this.angle = angle;
        this.rotation = rotation;
    }
}
