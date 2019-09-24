package org.firstinspires.ftc.teamcode.subsystems.imu;

/**
 * Created by Sarthak on 8/1/2018.
 */
public interface IIMU {

    /**
    * Gets the angle on the x-axis
    * @return the x-axis angle
    */
    double getXAngle();
    /**
     * Gets the angle on the y-axis
     * @return the y-axis angle
     */
    double getYAngle();
    /**
     * Gets the angle on the z-axis
     * @return the z-axis angle
     */
    double getZAngle();
    /**
     * Gets the angle on the z-axis with the discontinuity being opposite of the desired angle
     * @return the z-axis angle
     */
    double getZAngle(double desiredAngle);
    /**
     * Gets the acceleration on the x-axis
     * @return hte acceleration on the x-axis
     */
    double getXAcc();
    /**
     * Gets the acceleration on the y-axis
     * @return the accerlation on the y-axis
     */
    double getYAcc();
    /**
     * Gets the acceleration on the z-axis
     * @return the acceleration on the z-axis
     */
    double getZAcc();
    /**
     * Gets the velocity on the x-axis
     * @return the x-axis velocity
     */
    double getXVelo();
    /**
     * Gets the acceleration on the y-axis
     * @return the y-axis velocity
     */
    double getYVelo();
    /**
     * Gets the acceleration on the z-axis
     * @return the z-axis velocity
     */
    double getZVelo();

    double getXPos();

    double getYPos();

    double getZPos();

    /**
     * Calibrate the imu and initialize the parameters
     */
    void calibrate();

    /**
     * Set an offest for the imu
     * @param offset the offset to set
     */
    void setOffset(double offset);

    /**
     * Set the current position as the zero position for the imu
     */
    void setAsZero();

    /**
     * Initialize the parameters for the IMU
     */
    void initialize();
}
