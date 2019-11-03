package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
/*
Frame work for a mecanum/omni drive train, the implemented interfaces provide the additional variables and functions to make movement possible
 */
abstract public class HolonomicFourWheelDrivetrain extends Drivetrain implements Holonomic, Rotatable, Positionable
{
    private double rotation = 0; //the robots rotation about its own z axis
    private double course = 0; //the angle the robot is going to move at relative to its heading
    private double targetPosition = 0; //distance the robot has to move

    private double[] wheelTargetPositions = new double[4];
    private DcMotor.RunMode[] runModes = new DcMotor.RunMode[4];
    private final double[] wheelAngles;

    public HolonomicFourWheelDrivetrain(DcMotor[] motorList, double[] wheelAngles, boolean teleOpMode)
    {
        super(motorList, teleOpMode);
        this.wheelAngles = wheelAngles;
    }

    @Override
    public void setRotation(double rotation)
    {
        this.rotation = rotation;
        if (isTeleOpMode())
        {
            updateMotorPowers();
        }
    }

    @Override
    public double getRotation() { return rotation; }

    @Override
    public double getCourse() { return course; }

    @Override
    public void setCourse(double course)
    {
        this.course = course;
        if (isTeleOpMode())
        {
            updateMotorPowers();
        }
    }

    // passes the motor powers as calculated under the calculateWheelPower function back to the update motor powers function defined in Drivetrain
    protected double[] calculateMotorPowers()
    {
        double[] motorPowers = new double[this.motorList.length];
        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++)
        {
            motorPowers[motorIndex] = calculateWheelPower(course, getVelocity(), rotation, wheelAngles[motorIndex]);
        }
        return motorPowers;
    }

    // this is a multiplier since all mec wheels exert a certain amount of force on the robot being at an "angle"
    abstract double calculateWheelCoefficient(double course, double wheelAngle);

    // returns the power for each wheel based on the wheel coefficient and multiplying that by the velocity with the rotation
    private double calculateWheelPower(double course, double velocity, double rotationPower, double wheelAngle)
    {
        return calculateWheelCoefficient(course, wheelAngle)*velocity+rotationPower;
    }

    // takes in all the variables defined at the top of the class to set each motors target position then starts the movement process
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

    // returns not the robots actual position on the field but the distance moved based on the current movement goal
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

    @Override
    public double getTargetPosition()
    {
        return targetPosition;
    }

    //TODO: I believe this is the best place to implement a PID or acceleration curve for the motors
    @Override
    public void updatePosition()
    {
        /*
        Concept:
        constantly check the work done function to get a percentage of the distance traveled
        change the motor power such that at the ends ie furthers from 50% motor power is lowest
        motor power represents a percentage of the current velocity set by the functions

        either set the motor powers here directly or if the update motor powers function is running continuously (i dont think it is)
        update the velocity variable constantly (i don't think that is how it works though)

        upon further inspection that isn't how it works however we should be doing by updating the velocity function constantly
        save the passed velocity as target velocity somewhere possibly add to the list of variables defined in the upper levels

        the equation y = -((X - 0.5) * 2)^2 + 1.5 goes between 0.5 and 0.5 peaking at 1 in the middle for x [0, 1]
        for this instance x is the percentageTraveled and y is the velocity multiplier

        only activates when in auto mode
         */
    }

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

    // returns the motors to their prior state after resetting the encoders back to 0
    @Override
    public void finishPositioning()
    {
        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++)
        {
            //this.motorList[motorIndex].setPower(0); //Hopefully stops the drift at the end of move commands
            this.motorList[motorIndex].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motorList[motorIndex].setMode(runModes[motorIndex]);
        }
    }

    // executes the position functions
    @Override
    public void position()
    {
        //setMovementVelocity(getVelocity());
        while (isPositioning())
        {
            updatePosition();
        }
        finishPositioning();
    }

    // returns each motors ticks per full revolution
    @Override
    public double getTicksPerRev()
    {
        double ticksPerUnit = 0;
        for (DcMotor motor : this.motorList) ticksPerUnit += motor.getMotorType().getTicksPerRev();
        ticksPerUnit /= this.motorList.length;
        return ticksPerUnit;
    }

    // returns each motors ticks per in (its not actually ticks per in its really closer to ticks per ft for some reason this is fixed later in the mecanum robot class)
    @Override
    public double getTicksPerIn(double wheelRadius, double motorToWheelRatio)
    {
        return (getTicksPerRev()/(wheelRadius * motorToWheelRatio * 2 * Math.PI));
    }
}
