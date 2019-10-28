package org.firstinspires.ftc.teamcode.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.Testing.TeleOpTrollTest;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {

    private TeleOpTrollTest opMode;
    public BNO055IMU gyro;
    public Orientation angles;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    ModernRoboticsI2cRangeSensor uSonic;

    public void initSensors(TeleOpTrollTest opMode) {

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

        uSonic = this.opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Ultrasonic");
    }

    public double getDist () {
        return uSonic.getDistance(DistanceUnit.INCH);
    }

    public double MRConvert (double angle) {
        if (angle <= 0) {
            angle += 360;
        }
        return angle;
    }

    public double getGyroYaw() {
        return MRConvert(angles.firstAngle);
    }

    /*
    public double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
     */

    public double getGyroPitch() {
        return MRConvert(angles.secondAngle);
    }

    public double getGyroRoll() {
        return MRConvert(angles.thirdAngle);
    }
}
