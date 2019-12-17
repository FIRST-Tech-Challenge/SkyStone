package org.firstinspires.ftc.robotlib.drivetrain;

/**
 * A drivetrain whose heading is possible to set using a gyroscope.
 * Extends {@link Rotatable}, because all drivetrains with settable headings must be rotatable.
 */
public interface Odometrical extends Rotatable {
    /**
     * Set the target heading of the drivetrain.
     * @param targetHeading the angle that you want the drivetrain to move towards
     */
    void setTargetHeading(double targetHeading);

    /**
     * Get the current heading of the drivetrain (presumably a value from a sensor and not necessarily the drivetrain's target heading).
     * @return the angle the drivetrain is currently facing
     */
    double getCurrentHeading();

    /**
     * Get the target heading of the drivetrain (not necessarily the actual, current heading of the drivetrain), passed in using {@link #setTargetHeading}.
     * @return the heading that the robot is currently trying to get to or move along.
     */
    double getTargetHeading();

    /**
     * Recalculate motor powers to maintain or move towards the target heading
     */
    void updateHeading();
    void rotate();

    /**
     * Use this as a loop condition (with {@link #updateHeading in the loop body) if you want to turn to a specific heading and then move on to other code.
     * @return Whether or not the drivetrain is still rotating towards the target heading
     */
    boolean isRotating();

    /**
     * If there are any settings (motor {@link com.qualcomm.robotcore.hardware.DcMotor.RunMode RunModes}, etc.) you changed to rotate, change them back in this method.
     */
    void finishRotating();
}
