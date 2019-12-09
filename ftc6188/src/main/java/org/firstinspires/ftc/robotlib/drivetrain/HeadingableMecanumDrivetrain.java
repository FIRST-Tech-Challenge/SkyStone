package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotlib.motor.EncoderMotor;

public class HeadingableMecanumDrivetrain extends MecanumDrivetrain implements Headingable
{
    // Robots current heading
    private double course;

    // desired heading
    private double targetHeading = 0;

    private BNO055IMUImpl imu;

    public HeadingableMecanumDrivetrain(EncoderMotor[] motorList, double wheelRadius, double wheelToMotorRatio, BNO055IMUImpl imu)
    {
        super(motorList, wheelRadius, wheelToMotorRatio);
        this.imu = imu;
    }

    @Override
    public void setTargetHeading(double targetHeading)
    {
        this.targetHeading = targetHeading;
    }

    @Override
    public double getCurrentHeading()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    @Override
    public double getTargetHeading()
    {
        return targetHeading;
    }

    @Override
    public void updateHeading() { }

    @Override
    public void rotate()
    {
        double direction = Math.signum(getCurrentHeading() - targetHeading);
        this.setRotation(direction);

        updateMotorPowers();
        while (isRotating()) { updateHeading(); }

        finishRotating();
    }

    @Override
    public boolean isRotating()
    {
        return (getTargetHeading() - getCurrentHeading() <= 5);
    }

    @Override
    public void finishRotating()
    {
        this.finishPositioning();
    }
}
