package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotlib.motor.EncoderMotor;

/*
Frame work for a mecanum/omni drive train, the implemented interfaces provide the additional variables and functions to make movement possible
 */
abstract public class HolonomicFourWheelDrivetrain extends Drivetrain implements Holonomic, Rotatable, Positionable
{
    // Movement variables
    protected double rotation = 0; //the robots rotation about its own z axis
    private double course = 0; //the angle the robot is going to move at relative to its heading
    private double targetPosition = 0; //distance the robot has to move
    private double ticksPerIn = 0; //the number of encoder ticks needed to move the robot 1 inch

    // Motor information variables
    public double[] wheelTargetPositions = new double[4];
    private final double[] wheelAngles;
    private DcMotor.RunMode[] runModes = new DcMotor.RunMode[4];

    HolonomicFourWheelDrivetrain(EncoderMotor[] motorList, double[] wheelAngles)
    {
        super(motorList);
        this.wheelAngles = wheelAngles;
    }

    @Override
    public void setRotation(double rotation)
    {
        this.rotation = rotation;
        updateMotorPowers();
    }

    @Override
    public double getRotation() { return rotation; }

    @Override
    public double getCourse() { return course; }

    @Override
    public void setCourse(double course)
    {
        this.course = course;
        updateMotorPowers();
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

        for (DcMotor motor : this.motorList) { motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); }

        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++)
        {
            wheelTargetPositions[motorIndex] = targetPosition*calculateWheelCoefficient(course, wheelAngles[motorIndex]);
            this.motorList[motorIndex].setTargetPosition((int)(wheelTargetPositions[motorIndex]+0.5));
        }

        for (DcMotor motor : this.motorList) { motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); }
    }

    @Override
    public double getTargetPosition()
    {
        return targetPosition;
    }

    // returns not the robots actual position on the field but the distance moved based on the current movement goal
    @Override
    public double getCurrentPosition()
    {
        double amountDone = 0;
        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++)
        {
            amountDone += (double)this.motorList[motorIndex].getCurrentPosition()/wheelTargetPositions[motorIndex];
        }
        return amountDone/4.0 * getTargetPosition();
    }

    // auto function to initiate the positioning of the robot, dont use instead recreate loop in robotmove function
    @Override
    public void position()
    {
        double timeoutTime = getTargetPosition() * motorList[0].getMotorType().getAchieveableMaxTicksPerSecond();
        if (timeoutTime < 5) { timeoutTime *= 5; }

        // Creates a timer object so the robot will auto stop after 2 times the timeoutTime (seconds)
        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset();

        // do runs the inner code before checking against the while conditional this is needed since on first call velocity will be 0
        updateMotorPowers();
        do { updatePosition(); }
        while (isPositioning() && timeoutTimer.seconds() <= timeoutTime);
        finishPositioning();
    }

    // auto function to check if robot is still executing a movement command
    @Override
    public boolean isPositioning()
    {
        for (EncoderMotor motor : motorList) { if (motor.isEncoderBusy()) { return true; }}
        return false;
    }

    // auto function to implement an acceleration curve
    @Override
    public void updatePosition() { }

    // returns the motors to their prior state after resetting the encoders back to 0
    @Override
    public void finishPositioning()
    {
        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++)
        {
            runModes[motorIndex] = this.motorList[motorIndex].getMode();
            this.motorList[motorIndex].setPower(0); //Hopefully stops the drift at the end of move commands
            this.motorList[motorIndex].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motorList[motorIndex].setMode(runModes[motorIndex]);
        }

        // reset drive to a pre-movement state
        this.setVelocity(0);
        setCourse(0);
        setTargetPosition(0);
        setRotation(0);
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
    public void setTicksPerIn(double wheelRadius, double motorToWheelRatio)
    { this.ticksPerIn = (getTicksPerRev()/(wheelRadius * motorToWheelRatio * 2 * Math.PI)); }

    @Override
    public double getTicksPerIn() { return ticksPerIn; }
}
