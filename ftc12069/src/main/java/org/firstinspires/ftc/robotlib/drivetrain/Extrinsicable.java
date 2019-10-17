package org.firstinspires.ftc.robotlib.drivetrain;

public interface Extrinsicable
{
    void setExtrinsic(boolean extrinsic);
    boolean getExtrinsic();
    void setExtrinsicOffset(double extrinsicOffset);
    double getExtrinsicOffset();
    void updateCourse();
}
