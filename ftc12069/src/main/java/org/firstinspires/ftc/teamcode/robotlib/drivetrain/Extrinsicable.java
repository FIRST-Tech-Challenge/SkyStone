package org.firstinspires.ftc.teamcode.robotlib.drivetrain;

public interface Extrinsicable
{
    void setExtrinsic(boolean extrinsic);
    boolean getExtrinsic();
    void setExtrinsicOffset(double extrinsicOffset);
    double getExtrinsicOffset();
    void updateCourse();
}
