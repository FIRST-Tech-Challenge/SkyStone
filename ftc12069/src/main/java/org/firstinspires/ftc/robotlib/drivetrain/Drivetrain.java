package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract public class Drivetrain {
    private double velocity = 0;

    /**
     * Motors included in the drivetrain.
     */
    public DcMotor[] motorList;

    /**
     * The list of powers each of the corresponding motors in the {@link #motorList} array should be set to.
     */
    private double[] motorPowers;

    public Drivetrain(DcMotor[] motorList) { this.motorList = motorList; }

    /**
     * Retrieves the velocity of the robot
     * @return robot velocity
     */
    public double getVelocity() { return velocity; }

    /**
     * Sets the velocity of the robot
     * @param velocity new velocity
     */
    public void setVelocity(double velocity)
    {
        this.velocity = scaleVelocity(velocity);
        updateMotorPowers();
    }

    /**
     * Calculates the motor powers, then sets the motor powers to move the robot.
     */
    protected void updateMotorPowers()
    {
        motorPowers = calculateMotorPowers();
        for (int motorIndex = 0; motorIndex < motorPowers.length; motorIndex++)
        {
            motorList[motorIndex].setPower(motorPowers[motorIndex]);
        }
    }

    /**
     * Scales the velocity to allow for more precision
     * @param input raw velocity
     * @return scaled velocity
     */
    private double scaleVelocity(double input)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        int index = (int) (input * 16.0);

        if (index < 0) {
            index = -index;
        }

        if (index > 16) {
            index = 16;
        }

        double scaledInput;
        if (input < 0) {
            scaledInput = -scaleArray[index];
        } else {
            scaledInput = scaleArray[index];
        }

        return scaledInput;
    }

    /**
     * Calculates the motor powers then updates the motors
     * @return array of motor powers
     */
    abstract protected double[] calculateMotorPowers();
}
