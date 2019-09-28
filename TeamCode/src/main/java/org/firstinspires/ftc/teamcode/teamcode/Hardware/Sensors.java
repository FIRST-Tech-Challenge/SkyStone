package org.firstinspires.ftc.teamcode.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Sensors {

    private LinearOpMode opMode;
    public BNO055IMU gyro;
    public Orientation angles;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public void initSensors(LinearOpMode opMode) {

        this.opMode = opMode;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = this.opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

    }

    public double getGyroYaw() {

        return angles.firstAngle;
    }

    public double getGyroPitch() {

        return angles.secondAngle;
    }

    public double getGyroRoll() {

        return angles.thirdAngle;
    }
}
