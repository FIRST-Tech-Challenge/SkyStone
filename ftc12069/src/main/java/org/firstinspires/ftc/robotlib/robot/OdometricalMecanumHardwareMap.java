package org.firstinspires.ftc.robotlib.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotlib.controller.ErrorTimeThresholdFinishingAlgorithim;
import org.firstinspires.ftc.robotlib.controller.FinishableIntegratedController;
import org.firstinspires.ftc.robotlib.controller.PIDController;
import org.firstinspires.ftc.robotlib.drivetrain.OdometricalMecanumDrivetrain;
import org.firstinspires.ftc.robotlib.sensor.IntegratingGyroscopeSensor;

/**
 * Mecanum Hardware Map with sensors for movement in autonomous
 */
public class OdometricalMecanumHardwareMap extends MecanumHardwareMap {
    // REV IMU
    public BNO055IMUImpl imu;

    public FinishableIntegratedController controller;
    public OdometricalMecanumDrivetrain drivetrain;

    /**
     * Creates an odometrical mecanum hardware map
     * @param hwMap FTC hardware map
     */
    public OdometricalMecanumHardwareMap(HardwareMap hwMap) {
        super(hwMap);

        // Configure the REV HUB IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMUImpl.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());

        PIDController pid = new PIDController(1.5, 0.05, 0);
        pid.setMaxErrorForIntegral(0.002);

        controller = new FinishableIntegratedController(new IntegratingGyroscopeSensor(imu), pid, new ErrorTimeThresholdFinishingAlgorithim(Math.PI/50, 1));
        drivetrain = new OdometricalMecanumDrivetrain(motorList, controller);
    }
}
