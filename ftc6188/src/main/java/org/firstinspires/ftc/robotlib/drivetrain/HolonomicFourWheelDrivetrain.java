package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

abstract public class HolonomicFourWheelDrivetrain extends Drivetrain implements Holonomic, Rotatable, Positionable
{
    private double rotation = 0;
    private double course = 0;
    private double targetPosition = 0;

    public double[] wheelTargetPositions = new double[4];
    private DcMotor.RunMode[] runModes = new DcMotor.RunMode[4];
    private final double[] wheelAngles;

    public HolonomicFourWheelDrivetrain(DcMotor[] motorList, double[] wheelAngles)
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

    abstract double calculateWheelCoefficient(double course, double wheelAngle);

    private double calculateWheelPower(double course, double velocity, double rotationPower, double wheelAngle)
    {
        return calculateWheelCoefficient(course, wheelAngle)*velocity+rotationPower;
    }

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

    @Override
    public void updatePosition()
    {

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

    @Override
    public void finishPositioning()
    {
        for (DcMotor motor : this.motorList) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        for (int motorIndex = 0; motorIndex < this.motorList.length; motorIndex++) this.motorList[motorIndex].setMode(runModes[motorIndex]);
    }

    @Override
    public void position()
    {
        while (isPositioning()) updatePosition();
        finishPositioning();
    }

    @Override
    public double getTicksPerUnit()
    {
        double ticksPerUnit = 0;
        for (DcMotor motor : this.motorList) ticksPerUnit += motor.getMotorType().getTicksPerRev();
        ticksPerUnit /= this.motorList.length;
        return ticksPerUnit;
    }
}
