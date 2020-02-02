package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 This is an abstract class that handles 4 drive train motors.
 */
abstract class Movement extends LinearOpMode
{
    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    protected DcMotor backLeft;
    protected DcMotor backRight;

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
        frontLeft = hardwareMap.get(DcMotor.class, "leftfront");
        frontRight = hardwareMap.get(DcMotor.class, "rightfront");
        backRight = hardwareMap.get(DcMotor.class, "rightback");
        backLeft = hardwareMap.get(DcMotor.class, "leftback");

        // Most robots need the motor on one side to be reve`rsed to drive goBackward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        updateTelemetryMessage("Initialized Motors");

        arm = hardwareMap.get(DcMotor.class, "Arm");
        arm.setDirection(DcMotor.Direction.FORWARD);

        frontServo = hardwareMap.servo.get("frontServo");
        leftConstruction =  hardwareMap.servo.get("leftConstruction");
        rightConstruction = hardwareMap.servo.get("rightConstruction");
    }

    public void stop(final String message) {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
        backLeft.setPower(0.0);

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
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
        sleep(duration);

        updateTelemetryMessage("Going Forward");
    }

    // Backward is same as forward with reverse power
    public void goForward(final double power, final int duration) {
        goBackward(-power, duration);
    }

    public void strafeRight(final double power, final int duration) {
        //TODO - clarify how these motor powers are distributed for strafeRight movement
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(-power);
        backLeft.setPower(power);
        sleep(duration);

        updateTelemetryMessage("Strafing Left");
    }

    public void strafeLeft(final double power, final int duration) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(power);
        backLeft.setPower(-power);
        sleep(duration);

        updateTelemetryMessage("Strafing Right");
    }

    protected void updateTelemetryMessage(String message) {
        updateTelemetryMessage("Status", message);
    }

    protected void updateTelemetryMessage(String caption, String message) {
        telemetry.addData("Status", message);
        telemetry.update();
    }

    protected void turnRight(final double leftwheelsforwardpower, final double rightwheelsbackwardpower,  final int duration) {
        frontLeft.setPower(leftwheelsforwardpower);
        frontRight.setPower(-rightwheelsbackwardpower);
        backRight.setPower(-rightwheelsbackwardpower);
        backLeft.setPower(leftwheelsforwardpower);
        sleep(duration);

        updateTelemetryMessage("Turning Right");
    }

   protected void turnLeft(final double leftwheelsbackwardpower, final double rightwheelsforwardpower, final int duration) {
        frontLeft.setPower(-leftwheelsbackwardpower);
        frontRight.setPower(rightwheelsforwardpower);
        backRight.setPower(rightwheelsforwardpower);
        backLeft.setPower(-leftwheelsbackwardpower);
        sleep(duration);

        updateTelemetryMessage("Turning Left");
    }

    public void armUp(final double armpower, final int duration) {
        arm.setPower(armpower);
        sleep(duration);

        updateTelemetryMessage("Arm going up");
    }

    public void armDown(final double armpower, final int duration) {
        armUp(-armpower, duration);
    }


    public void armClamp() {
        frontServo.setPosition(0.0);
        sleep(200);
    }

    public void armRelease() {
        frontServo.setPosition(0.4);
        sleep(200);
   }



    public void backServosDown() {
        rightConstruction.setPosition(0.43);
        leftConstruction.setPosition(0.35);

        updateTelemetryMessage("Foundation Servos Down");
    }

    public void backServosUp() {
        rightConstruction.setPosition(1);
        leftConstruction.setPosition(1);

        updateTelemetryMessage("Foundation Servos Up");
    }
}