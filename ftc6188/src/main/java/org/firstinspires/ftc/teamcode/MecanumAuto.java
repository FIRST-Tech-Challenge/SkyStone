package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class MecanumAuto extends LinearOpMode
{
    private MecanumHardwareMap robotHardware;
    private ElapsedTime elapsedTime;

    private double wheelTargetPositions;
    private double direction;
    private double rotation;
    private double velocity;
    private double targetPosition;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotHardware = new MecanumHardwareMap(this.hardwareMap);
        elapsedTime = new ElapsedTime();

        waitForStart();

    }


    private void setRotation(double rotation)
    {
        this.rotation = rotation;

    }

    private void setDirection(double direction)
    {
        this.direction = direction;

    }

    private double[] calculateMotorPowers()
    {
        double[] motorPowers = new double[4];
        for (int motorIndex = 0; motorIndex < 4; motorIndex+=1)
        {
            motorPowers[motorIndex] = calculateWheelPower(direction, velocity, rotation, robotHardware.wheelAngles[motorIndex]);
            robotHardware.motorList[motorIndex].setPower(motorPowers[motorIndex]);
        }
        return motorPowers;
    }

    private double calculateWheelPower(double direction, double velocity, double rotationPower, double wheelAngle)
    {
        return (calculateWheelCoefficient(direction, wheelAngle))*velocity+rotationPower;
    }

    private double calculateWheelCoefficient(double direction, double wheelAngle)
    {
        return (Math.cos(direction)-Math.sin(direction)/Math.tan(wheelAngle))*Math.signum(wheelAngle);
    }

    private double setTargetPosition(double targetPosition)
    {
        for (int motorIndex = 0; motorIndex < robotHardware.runModes.length; motorIndex+=1)
        {
            robotHardware.runModes[motorIndex] = robotHardware.motorList[motorIndex].getMode();
        }
        this.targetPosition = targetPosition;

        for (DcMotor motor : robotHardware.motorList)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for (DcMotor motor : robotHardware.motorList)
        {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (int motorIndex = 0; motorIndex < robotHardware.motorList.length; motorIndex+=1)
        {
            robotHardware.wheelTargetPositions[motorIndex] = targetPosition*calculateWheelCoefficient(direction, robotHardware.wheelAngles[motorIndex]);
            robotHardware.motorList[motorIndex].setTargetPosition((int)(robotHardware.wheelTargetPositions[motorIndex]+0.5));
        }
    }

    private void updateMotorPowers()
    {

    }
}
