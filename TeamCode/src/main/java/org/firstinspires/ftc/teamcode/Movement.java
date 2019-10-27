package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 This is an abstrcat class that handles 4 drive train motors.
 */
abstract class Movement extends LinearOpMode
{
    protected DcMotor leftfront;
    protected DcMotor rightfront;
    protected DcMotor leftback;
    protected DcMotor rightback;

    public void runOpMode() {
        setupDriveMotors();
        runOpModeImpl();
    }

    public abstract void runOpModeImpl();

    protected void setupDriveMotors() {
        // Initialize the motor references for all the wheels
        // Initialize the hardware variables. Note that the strings used here as parameters

        updateTelemetry("Initializing Motors");
        leftfront  = hardwareMap.get(DcMotor.class, "leftfront");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        leftback = hardwareMap.get(DcMotor.class, "leftback");

        // Most robots need the motor on one side to be reversed to drive goForward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        updateTelemetry("Initialized Motors");
    }

    public void stop(final String message) {
        leftfront.setPower(0.0);
        rightfront.setPower(0.0);
        rightback.setPower(0.0);
        leftback.setPower(0.0);

        updateTelemetry(message);
    }

    public void goForward(final double power, final int duration, final String message) {
        leftfront.setPower(power);
        rightfront.setPower(power);
        rightback.setPower(power);
        leftback.setPower(power);
        sleep(duration);

        updateTelemetry(message);
    }

    public void goBackward(final double power, final int duration, final String message) {

        leftfront.setPower(-power);
        rightfront.setPower(-power);
        rightback.setPower(-power);
        leftback.setPower(-power);
        sleep(duration);

        updateTelemetry(message);
    }

    public void goLeft(final double power, final int duration, final String message) {
        //TODO - clarify how these motor powers are distributed for goLeft movement
        leftfront.setPower(-power);
        rightfront.setPower(power);
        rightback.setPower(-power);
        leftback.setPower(power);
        sleep(duration);

        updateTelemetry(message);
    }

    public void goRight(final double power, final int duration, final String message) {
        leftfront.setPower(power);
        rightfront.setPower(-power);
        rightback.setPower(power);
        leftback.setPower(-power);
        sleep(duration);

        updateTelemetry(message);
    }

    private void updateTelemetry(String message) {
        telemetry.addData("Status", message);
        telemetry.update();
    }
}