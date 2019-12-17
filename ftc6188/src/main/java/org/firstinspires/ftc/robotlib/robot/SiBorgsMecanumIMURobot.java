package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.drivetrain.HeadingableMecanumDrivetrain;
import org.jetbrains.annotations.NotNull;

public class SiBorgsMecanumIMURobot extends SiBorgsMecanumRobot
{
    // Orientation/accel sensor
    private BNO055IMUImpl imu;

    // Drivetrain
    public HeadingableMecanumDrivetrain drivetrain;

    public SiBorgsMecanumIMURobot(@NotNull HardwareMap hwMap, Telemetry telemetry)
    {
        super(hwMap, telemetry);

        /** INIT IMU **/
        imu = hwMap.get(BNO055IMUImpl.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        /** INIT DRIVETRAIN **/
        drivetrain = new HeadingableMecanumDrivetrain(this.driveMotorList, 2, 2, imu);
    }

    public void angleTelemetry()
    {
        this.telemetry.addData("Current Angle", drivetrain.getCurrentHeading());
        this.telemetry.addData("Target Angle", drivetrain.getTargetHeading());
        this.telemetry.addData("Target Delta", drivetrain.getTargetDelta());
        this.telemetry.addData("IsRotating", drivetrain.isRotating());
        this.telemetry.update();
    }
}
