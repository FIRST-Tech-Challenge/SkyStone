package org.firstinspires.ftc.teamcode.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract public class HolonomicFourWheelDrivetrain extends Drivetrain implements Holonomic, Rotatable, Positionable
{
    private double rotation = 0;
    private double course = 0;
    private double targetPosition = 0;

    /**
     * Stores how far each wheel has to go to get the drivetrain to a specific position
     */
    private double[] wheelTargetPositions = new double[4];

    /**
     * The RunModes of each of the motors (used after changing the RunModes to move to a position)
     */
    private DcMotor.RunMode[] runModes = new DcMotor.RunMode[4];

    /**
     * A list of angles related to wheel and drivetrain geometry that must be defined by subclasses
     */
    private final double[] wheelAngles;

    /**
     * Constructor for a generic holonomic four wheel drivetrain
     * @param motorList The array of motors included in the drivetrain
     * @param wheelAngles A list of four angles corresponding to each wheel related to wheel and drivetrain geometry; these should be defined by the subclass and are passed to {@link #calculateWheelCoefficient} (which also must be defined by the subclass)
     */
    public HolonomicFourWheelDrivetrain(DcMotor[] motorList, double[] wheelAngles)
    {
        super(motorList);
        this.wheelAngles = wheelAngles;
    }

    /**
     * Sets the drivetrain's angular (rotational) velocity.
     * @param rotation The velocity that you want to rotate the robot at, between -1 (full power clockwise) and 1 (full power counterclockwise) — 0 for no rotation
     */
    @Override
    public void setRotation(double rotation)
    {
        this.rotation = rotation;
        updateMotorPowers();
    }

    /**
     * Gets the drivetrain's target angular (rotational) velocity.
     * @return the rotation velocity the robot was given, between -1 (full power clockwise) and 1 (full power counterclockwise) — 0 for no rotation
     */
    @Override
    public double getRotation() { return rotation; }


    /**
     * Gets the direction the robot is supposed to be moving along.
     * @return The course in radians, where 0 is forwards and {@link Math#PI}/2 is directly to the left.
     */
    @Override
    public double getCourse() { return course; }

    /**
     * Sets the direction you want the robot to move along.
     * @param course The course in radians, where 0 is forwards and {@link Math#PI}/2 is directly to the left.
     */
    @Override
    public void setCourse(double course)
    {
        this.course = course;
        updateMotorPowers();
    }

    /**
     * Re-calculate the powers for each of the motors (called after a velocity, rotation, or course change) and update motors
     * @return an array of motor powers
     */
    protected double[] calculateMotorPowers()
    {
        double[] motorPowers = new double[this.motorList.length];
        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++)
        {
            motorPowers[motorIndex] = calculateWheelPower(course, getVelocity(), rotation, wheelAngles[motorIndex]);
            this.motorList[motorIndex].setPower(motorPowers[motorIndex]);
        }
        return motorPowers;
    }

    /**
     * Abstract method for calculating specific wheel velocity percentage
     * @param course the angle that you want the robot to move
     * @param wheelAngle the array of angles that the actual moving parts of the wheels are at
     * @return a number between zero and one, which says what percentage of the speed a wheel should move at. Is then multiplied by the velocity
     */
    abstract double calculateWheelCoefficient(double course, double wheelAngle);

    /**
     * Takes a wheel coefficient from the subclass and calculates a wheel power from it.
     * @param course the angle that you want the robot to more along
     * @param velocity the velocity you want the robot to move with
     * @param rotationPower the velocity that you want to rotate the robot by.
     *                 Counterclockwise is positive and clockwise is negative.
     *                 Zero is if you don't want to rotate the robot.
     * @param wheelAngle the angle of the actual moving part of the wheel
     * @return the power the motor is supposed to move with, which is then sent to the motor
     */
    private double calculateWheelPower(double course, double velocity, double rotationPower, double wheelAngle)
    {
        return calculateWheelCoefficient(course, wheelAngle)*velocity+rotationPower;
    }

    /**
     * Sets target position and handles potentially new motor velocities
     * @param targetPosition the position that you want the robot to move to
     */
    @Override
    public void setTargetPosition(double targetPosition)
    {
        for (int motorIndex = 0; motorIndex < this.runModes.length; motorIndex++)
        {
            runModes[motorIndex] = this.motorList[motorIndex].getMode();
        }
        this.targetPosition = targetPosition;

        for (DcMotor motor : this.motorList) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        for (DcMotor motor : this.motorList) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++)
        {
            wheelTargetPositions[motorIndex] = targetPosition*calculateWheelCoefficient(course, wheelAngles[motorIndex]);
            this.motorList[motorIndex].setTargetPosition((int)(wheelTargetPositions[motorIndex]+0.5));
        }
        updateMotorPowers();
    }

    /**
     * Retrieves the current position of the robot relative to the target
     * @return the current position of the robot
     */
    @Override
    public double getCurrentPosition()
    {
        double amountDone = 0;
        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++)
        {
            amountDone += this.motorList[motorIndex].getCurrentPosition()/wheelTargetPositions[motorIndex];
        }
        return amountDone/4*targetPosition;
    }

    /**
     * @return the position the robot is supposed to move to (the Target Position)
     */
    @Override
    public double getTargetPosition()
    {
        return targetPosition;
    }

    @Override
    public void updatePosition()
    {
        //throw new UnsupportedOperationException("Position should be handled by the built-in controller");
    }

    /**
     * Use this as a loop condition (with {@link #updatePosition in the loop body) if you want to move to a specific position and then move on to other code.
     * If the drivetrain never seems to stop positioning, use {@link com.qualcomm.robotcore.hardware.DcMotorEx#setTargetPositionTolerance} on each of your motors to make them less perfectionistic.
     * @return Whether or not the drivetrain is still moving towards the target position
     */
    @Override
    public boolean isPositioning()
    {
        for (DcMotor motor : this.motorList)
        {
            if (motor.isBusy())
            {
                return true;
            }
        }
        return false;
    }

    /**
     * Resets encoders and changes the {@link DcMotor.RunMode RunModes} to what they were before
     */
    @Override
    public void finishPositioning()
    {
        for (DcMotor motor : this.motorList) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++) this.motorList[motorIndex].setMode(runModes[motorIndex]);
    }

    /**
     * Blocks the current thread until the robot is done positioning
     */
    @Override
    public void position()
    {
        while (isPositioning()) updatePosition();
        finishPositioning();
    }

    /**
     * Depending on the encoder type, the number of ticks per rotation can vary. This method uses motor configuration data to calculate that number.
     * @return the number of encoder ticks per rotation
     */
    @Override
    public double getTicksPerUnit()
    {
        double ticksPerUnit = 0;
        for (DcMotor motor : this.motorList) ticksPerUnit += motor.getMotorType().getTicksPerRev();
        ticksPerUnit /= this.motorList.length;
        return ticksPerUnit;
    }

    /**
     * Provides the number of motor "ticks" required to cover X inches
     * @param wheelRadius physical wheel radius
     * @param motorToWheelRatio motor to wheel ratio
     * @return number of motor "ticks" for X inches
     */
    public double getTicksPerInch(double wheelRadius, double motorToWheelRatio)
    {
        return (wheelRadius * motorToWheelRatio * 2 * Math.PI) / getTicksPerUnit();
    }

    /**
     * Provides the wheel rotation values for a velocity
     * @param velocity rotation speed (between 0 and 1)
     * @return Array of wheel rotation velocities
     */
    public double[] getWheelRotationValues(double velocity) {
        return new double[] {
                velocity > 0 ? velocity : -velocity, // Front Left
                velocity > 0 ? -velocity : velocity, // Front Right
                velocity > 0 ? velocity : -velocity, // Rear Right
                velocity > 0 ? -velocity : velocity // Rear Left
        };
    }
}
