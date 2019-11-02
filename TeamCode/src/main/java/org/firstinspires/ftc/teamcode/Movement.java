package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 This is an abstrcat class that handles 4 drive train motors.
 */
abstract class Movement extends LinearOpMode
{
    protected DcMotor leftfront;
    protected DcMotor rightfront;
    protected DcMotor leftback;
    protected DcMotor rightback;

    protected DcMotor arm;

    protected Servo frontServo;
    protected Servo backServo;

    public void runOpMode() {
        setupDriveMotors();
        runOpModeImpl();
    }

    public abstract void runOpModeImpl();

    protected void setupDriveMotors() {
        // Initialize the motor references for all the wheels
        // Initialize the hardware variables. Note that the strings used here as parameters

        updateTelemetryMessage("Initializing Motors");
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

        updateTelemetryMessage("Initialized Motors");

        arm = hardwareMap.get(DcMotor.class, "Arm");
        arm.setDirection(DcMotor.Direction.FORWARD);

        // TODO: Set up front servo motor
        //frontServo = hardwareMap.servo.get("");

        backServo =  hardwareMap.servo.get("servo");
    }

    public void stop(final String message) {
        leftfront.setPower(0.0);
        rightfront.setPower(0.0);
        rightback.setPower(0.0);
        leftback.setPower(0.0);

        updateTelemetryMessage(message);
    }

    public void goForward(final double power, final int duration, final String message) {
        leftfront.setPower(power);
        rightfront.setPower(power);
        rightback.setPower(power);
        leftback.setPower(power);
        sleep(duration);

        updateTelemetryMessage(message);
    }

    // Backward is same as forward with reverse power
    public void goBackward(final double power, final int duration, final String message) {
        goForward(-power, duration, message);
    }

    public void goLeft(final double power, final int duration, final String message) {
        //TODO - clarify how these motor powers are distributed for goLeft movement
        leftfront.setPower(-power);
        rightfront.setPower(power);
        rightback.setPower(-power);
        leftback.setPower(power);
        sleep(duration);

        updateTelemetryMessage(message);
    }

    public void goRight(final double power, final int duration, final String message) {
        leftfront.setPower(power);
        rightfront.setPower(-power);
        rightback.setPower(power);
        leftback.setPower(-power);
        sleep(duration);

        updateTelemetryMessage(message);
    }

    protected void updateTelemetryMessage(String message) {
        updateTelemetryMessage("Status", message);
    }

    protected void updateTelemetryMessage(String caption, String message) {
        telemetry.addData("Status", message);
        telemetry.update();
    }

}