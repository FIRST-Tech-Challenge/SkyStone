package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.motor.EncoderMotor;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;
/*
Drivetrain is the base class for all other drivetrains, each contain instructions on how the robot should move the wheels based on specific variables
 */
abstract public class Drivetrain
{
    private double velocity = 0;

    // Toggles the half power drive mode for precision control
    private ToggleBoolean lowPower;

    public EncoderMotor[] motorList;

    Drivetrain(EncoderMotor[] motorList)
    {
        this.motorList = motorList;
        lowPower = new ToggleBoolean(false);
    }

    Drivetrain(DcMotor[] motorList)
    {
        EncoderMotor[] encoderMotorList = new EncoderMotor[4];
        for (int motorIndex = 0; motorIndex < encoderMotorList.length; motorIndex++) { encoderMotorList[motorIndex] = new EncoderMotor(motorList[motorIndex]); }
        this.motorList = encoderMotorList;
        lowPower = new ToggleBoolean(false);
    }

    public double getVelocity() { return velocity; }

    public void setVelocity(double velocity)
    {
        this.velocity = velocity;
        updateMotorPowers();
    }

    // This function iterates through the list of motors and sets each power to the power calculated in the calculateMotorPowers function
    protected void updateMotorPowers()
    {
        double[] motorPowers = calculateMotorPowers();
        for (int motorIndex = 0; motorIndex < motorPowers.length; motorIndex++)
        {
            motorList[motorIndex].setPower(motorPowers[motorIndex] * (getLowPower() ? 0.35 : 1));
        }
    }

    public void lowPowerInput(boolean currentlyPressed)
    {
        lowPower.input(currentlyPressed);
    }
	
	public boolean getLowPower()
	{
		return lowPower.output();
	}

	// Defined per drivetrain, does math related to the power of the motor based on stick inputs
    abstract protected double[] calculateMotorPowers();
}
