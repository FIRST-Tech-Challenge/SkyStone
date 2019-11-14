package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.controller.ErrorTimeThresholdFinishingAlgorithim;
import org.firstinspires.ftc.robotlib.controller.FinishableIntegratedController;
import org.firstinspires.ftc.robotlib.controller.PIDController;
import org.firstinspires.ftc.robotlib.drivetrain.HeadingableMecanumDrivetrain;
import org.firstinspires.ftc.robotlib.sensor.IntegratingGyroscopeSensor;

public class HeadingableMecanumRobot extends MecanumRobot
{
    public BNO055IMUImpl imu;

    public FinishableIntegratedController controller;
    public HeadingableMecanumDrivetrain drivetrain;

    public HeadingableMecanumRobot(HardwareMap hwMap, Telemetry telemetry, boolean teleOpMode)
    {
        super(hwMap, telemetry, teleOpMode);

        imu = hwMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        PIDController pid = new PIDController(1.5, 0.05, 0);
        pid.setMaxErrorForIntegral(0.002);

        controller = new FinishableIntegratedController(new IntegratingGyroscopeSensor(imu), pid, new ErrorTimeThresholdFinishingAlgorithim(Math.PI/50, 1));
        this.drivetrain = new HeadingableMecanumDrivetrain(motorList, controller, teleOpMode, wheelRadius, wheelToMotorRatio);
    }
}
