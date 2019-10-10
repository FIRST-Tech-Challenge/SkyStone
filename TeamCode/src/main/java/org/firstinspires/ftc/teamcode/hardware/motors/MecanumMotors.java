package org.firstinspires.ftc.teamcode.hardware.motors;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.util.math.Vector;

public class MecanumMotors extends MotorGroup {

    //the stored velocities
    private double velocityX = 0;
    private double velocityY = 0;
    private double velocityR = 0;

    //the drive mode

    /**
     * @param motors          the motors
     * @param maxRobotSpeed   the measured speed of the robot at 100% power
     */
    public MecanumMotors(List<? extends Motor> motors, double maxRobotSpeed) {
        super(motors, maxRobotSpeed);
    }

    /**
     * run the motors based on the xyr velocities
     * normalize if any motor power is too large
     */
    private void moveMecanum() {
        //calculate motor powers
        double fl = velocityX + velocityY - velocityR;
        double fr = velocityX - velocityY + velocityR;
        double bl = velocityX - velocityY - velocityR;
        double br = velocityX + velocityY + velocityR;

        moveNormalized(Arrays.asList(fl, fr, bl, br));
    }

    /**
     * @param velocityX the x velocity
     */
    public void setVelocityX(double velocityX) {
        this.velocityX = velocityX;
    }

    /**
     * @param velocityY the y velocity
     */
    public void setVelocityY(double velocityY) {
        this.velocityY = velocityY;
    }

    /**
     * @param velocityR the rotational velocity
     */
    public void setVelocityR(double velocityR) {
        this.velocityR = velocityR;
    }

    /**
     * set the x and y velocities at the same time
     *
     * @param velocityX the x velocity
     * @param velocityY the y velocity
     */
    public void setVelocityXY(double velocityX, double velocityY) {
        setVelocityX(velocityX);
        setVelocityY(velocityY);
    }

    /**
     * set the x, y, and rotational velocities at the same time
     *
     * @param velocityX the x velocity
     * @param velocityY the y velocity
     * @param velocityR the rotational velocity
     */
    public void setVelocityXYR(double velocityX, double velocityY, double velocityR) {
        setVelocityX(velocityX);
        setVelocityY(velocityY);
        setVelocityR(velocityR);
    }

    /**
     * set the x and y velocities using polar coordinates
     *
     * @param vector the vector in which to move
     */
    public void setVelocityVector(Vector vector) {
        double directionRadians = vector.angle.radians();
        double velocity = vector.magnitude;

        setVelocityX(velocity * Math.cos(directionRadians));
        setVelocityY(velocity * Math.sin(directionRadians));
    }
}