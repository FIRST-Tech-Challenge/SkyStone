package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/*
    Use this an object of this class, if you need to access the imu (also called gyro) sensor.
    If you need to drive using the gyro look in the GyroTools class in the tools package.
    There we already created methods.
 */
public class HardwareChassisGyro {
    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    private HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareChassisGyro(HardwareMap hwMap){
        init(hwMap);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        Acceleration a = imu.getAcceleration();



    }
}
