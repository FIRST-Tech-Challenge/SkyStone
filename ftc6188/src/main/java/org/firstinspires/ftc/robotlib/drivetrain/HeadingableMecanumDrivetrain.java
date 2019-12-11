package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotlib.motor.EncoderMotor;

public class HeadingableMecanumDrivetrain extends MecanumDrivetrain implements Headingable
{
    // Robots current heading
    private double course;
    private double predictedHeading;

    // desired heading
    private double targetHeading = 0;

    // rotation
    private double rotationPower = 0;

    // IMU
    private BNO055IMUImpl imu;

    // Timer
    private ElapsedTime elapsedTime;

    public HeadingableMecanumDrivetrain(EncoderMotor[] motorList, double wheelRadius, double wheelToMotorRatio, BNO055IMUImpl imu)
    {
        super(motorList, wheelRadius, wheelToMotorRatio);
        this.imu = imu;
        this.elapsedTime = new ElapsedTime();
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
        double direction = Math.signum(targetHeading - getCurrentHeading());
        this.setRotation(direction * rotationPower);

        elapsedTime.reset();
        predictedHeading = 0;

        updateMotorPowers();
        while (isRotating()) { updateHeading(); }

        finishRotating();
    }

    @Override
    public boolean isRotating()
    {
        predictedHeading += getCurrentHeading()/elapsedTime.seconds();
        elapsedTime.reset();
        return !((int)predictedHeading == (int)targetHeading);
    }

    @Override
    public void finishRotating()
    {
        setVelocity(0);
        setRotation(0);
        for (DcMotor motor : motorList) { motor.setPower(0); }
    }

    public void autoRotate(double targetHeading, double velocity)
    {
        setTargetHeading(targetHeading);
        rotationPower = velocity;
        rotate();
    }
}
