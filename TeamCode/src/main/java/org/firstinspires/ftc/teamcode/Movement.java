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
    protected Servo rightConstruction;
    protected Servo leftConstruction;

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

        // Most robots need the motor on one side to be reve`rsed to drive goBackward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);

        updateTelemetryMessage("Initialized Motors");

        arm = hardwareMap.get(DcMotor.class, "Arm");
        arm.setDirection(DcMotor.Direction.FORWARD);

        frontServo = hardwareMap.servo.get("frontServo");
        leftConstruction =  hardwareMap.servo.get("leftConstruction");
        rightConstruction = hardwareMap.servo.get("rightConstruction");
    }

    public void stop(final String message) {
        leftfront.setPower(0.0);
        rightfront.setPower(0.0);
        rightback.setPower(0.0);
        leftback.setPower(0.0);

        updateTelemetryMessage(message);
    }

    public void stopWithSleep(final String message, final long duration) {
        stop(message);
        sleep(duration);
    }

    public void stopWithSleep(final long duration) {
        stop("Stopping");
        sleep(duration);
    }

    public void goBackward(final double power, final int duration) {
        leftfront.setPower(power);
        rightfront.setPower(power);
        rightback.setPower(power);
        leftback.setPower(power);
        sleep(duration);

        updateTelemetryMessage("Going Forward");
    }

    // Backward is same as forward with reverse power
    public void goForward(final double power, final int duration) {
        goBackward(-power, duration);
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
    protected void Turnright(final double leftwheelsforwardpower, final double rightwheelsbackwardpower,  final int duration) {
        leftfront.setPower(leftwheelsforwardpower);
        rightfront.setPower(-rightwheelsbackwardpower);
        rightback.setPower(-rightwheelsbackwardpower);
        leftback.setPower(leftwheelsforwardpower);
        sleep(duration);
    }

    protected void Turnleft(final double leftwheelsbackwardpower, final double rightwheelsforwardpower,  final int duration) {
        leftfront.setPower(-leftwheelsbackwardpower);
        rightfront.setPower(rightwheelsforwardpower);
        rightback.setPower(rightwheelsforwardpower);
        leftback.setPower(-leftwheelsbackwardpower);
        sleep(duration);
    }

    public void Armup(final double armpower, final int duration) {
        arm.setPower(armpower);
        sleep(duration);
        updateTelemetryMessage("Arm going up");
    }

    public void Armdown(final double armpower, final int duration) {
        Armup(-armpower, duration);
    }

    public void armclamp() {
        frontServo.setPosition(0.0);
        sleep(200);
    }

    public void armrelease() {
        frontServo.setPosition(0.4);
        sleep(200);
    }
}