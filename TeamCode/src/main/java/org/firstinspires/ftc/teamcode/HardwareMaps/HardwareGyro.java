package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by dreadjack on 28.11.17.
 *
 *
 *
 * To start collecting any data from the BNO055IMU use the following command:
 *
 * gyro.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
 *
 * assuming that you declared your class with the name "gyro"
 *
 */

public class HardwareGyro {
    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareGyro(HardwareMap hwMap){
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


    }

}
