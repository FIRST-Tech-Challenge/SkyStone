package org.firstinspires.ftc.robotlib.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.state.ToggleBoolean;
/*
Drivetrain is the base class for all other drivetrains, each contain instructions on how the robot should move the wheels based on specific variables
 */
abstract public class Drivetrain
{
    private double velocity = 0;
    private double movementVelocity = 0; // used to store desired max velocity during a movement command in auto

    // Toggles the half power drive mode for precision control
    private ToggleBoolean halfPower;

    // Since auto code messes up we have to tell the drivetrain if we are driving this manually or pre-programmed
    private final boolean teleOpMode;

    // Telemetry reference for auto code live view
    private Telemetry telemetry;

    public DcMotor[] motorList;

    public Drivetrain(DcMotor[] motorList, boolean teleOpMode)
    {
        this.motorList = motorList;
        this.teleOpMode = teleOpMode;
        this.telemetry = null;
        halfPower = new ToggleBoolean(false);
    }

    public Drivetrain(DcMotor[] motorList, boolean teleOpMode, Telemetry telemetry)
    {
        //TODO: Remove this and all other instances of telemetry within drivetrains (not necessary just for testing temporarily)
        this.motorList = motorList;
        this.teleOpMode = teleOpMode;
        this.telemetry = telemetry;
        halfPower = new ToggleBoolean(false);
    }

    public double getVelocity() { return velocity; }

    public void setVelocity(double velocity)
    {
        this.velocity = velocity;
        if (isTeleOpMode())
        {
            updateMotorPowers();
        }
    }

    public Telemetry getTelemetry()
    {
        return telemetry;
    }

    public double getMovementVelocity() { return movementVelocity; }

    public void setMovementVelocity(double movementVelocity) { this.movementVelocity = movementVelocity; }

    // This function iterates through the list of motors and sets each power to the power calculated in the calculateMotorPowers function
    protected void updateMotorPowers()
    {
        double[] motorPowers = calculateMotorPowers();
        for (int motorIndex = 0; motorIndex < motorPowers.length; motorIndex++)
        {
            motorList[motorIndex].setPower(motorPowers[motorIndex] * (halfPower.output() ? 0.5 : 1));
        }
    }

    public void halfPowerInput(boolean currentlyPressed)
    {
        halfPower.input(currentlyPressed);
    }
	
	public boolean isHalfPower()
	{
		return halfPower.output();
	}

	// Defined per drivetrain, does math related to the power of the motor based on stick inputs
    abstract protected double[] calculateMotorPowers();

    public boolean isTeleOpMode() { return teleOpMode; }
}
