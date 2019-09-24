package org.firstinspires.ftc.teamcode.subsystems.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Rohan on 8/1/2018.
 */
public class BoschIMU implements IIMU{

    //Create imu hardware
    BNO055IMU imu;
    double offset;

    /**
     * Constructor for Bosch IMU
     * @param imu imu sensor
     */
    public BoschIMU(BNO055IMU imu){
        this.imu = imu;
    }

    /**
     * Gets the angle on the x-axis
     * @return angle on the x-axis
     */
    @Override
    public double getXAngle() {
        return -imu.getAngularOrientation().thirdAngle - offset;
    }

    /**
     * Gets the angle on the y-axis
     * @return angle on the y-axis
     */
    @Override
    public double getYAngle() {
        return -imu.getAngularOrientation().secondAngle - offset;
    }

    /**
     * Gets the angle on the z-axis
     * @return angle on the z-axis
     */
    @Override
    public double getZAngle() {
        return -imu.getAngularOrientation().firstAngle - offset;
    }
    /**
     * Gets the angle on the z-axis while placing the imu discontinuity 180 degrees from the desired angle
     * @return angle on the z-axis
     */
    @Override
    public double getZAngle(double desiredAngle) {
        double angle = getZAngle();
        if(angle<desiredAngle-180){
            angle+=360;
        }else if(angle>desiredAngle+180){
            angle-=360;
        }
        return angle;
    }

    /**
     * Gets the acceleration on the x-axis
     * @return acceleration on the x-axis
     */
    @Override
    public double getXAcc() {
        return imu.getAcceleration().xAccel;
    }

    /**
     * Gets the acceleration on the y-axis
     * @return acceleration on the y-axis
     */
    @Override
    public double getYAcc() {
        return imu.getAcceleration().yAccel;
    }

    /**
     * Gets the acceleration on the z-axis
     * @return acceleration on the z-axis
     */
    @Override
    public double getZAcc() {
        return imu.getAcceleration().zAccel;
    }

    /**
     * Gets the velocity on the x-axis
     * @return velocity on the x-axis
     */
    @Override
    public double getXVelo() {
        return imu.getVelocity().xVeloc;
    }

    /**
     * Gets the velocity on the y-axis
     * @return velocity on the y-axis
     */
    @Override
    public double getYVelo() {
        return imu.getVelocity().yVeloc;
    }

    /**
     * Gets the velocity on the z-axis
     * @return velocity on the z-axis
     */
    @Override
    public double getZVelo() { return imu.getVelocity().zVeloc; }

    @Override
    public double getXPos()
    {
        return imu.getPosition().x;
    }

    @Override
    public double getYPos()
    {
        return imu.getPosition().y;
    }

    @Override
    public double getZPos()
    {
        return imu.getPosition().z;
    }

    /**
     * Calibrates IMU and initializes parameters
     */
    @Override
    public void calibrate() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IIMU";
        imu.initialize(parameters);
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }

    /**
     * Initializes IMU parameters
     */
    @Override
    public void initialize(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    /**
     * Sets an offset for the IMU angle values
     * @param offset the offset to set
     */
    @Override
    public void setOffset(double offset) {
        this.offset = offset;
    }

    /**
     * Set the current angle as 0
     */
    @Override
    public void setAsZero() {
        offset = -imu.getAngularOrientation().firstAngle;
    }

}
