package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SourceFiles.Trobot;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic POV Teleop for a four wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "POV Mode", group = "POV Mode")
public class TeleOp_FTC extends LinearOpMode {
    private ElapsedTime runtime;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor rearLeftDrive;
    private DcMotor rearRightDrive;

    private DcMotor leftIntake;
    private DcMotor rightIntake;

    private Servo leftLatch;
    private Servo rightLatch;

    String latchStatus = "Pending";

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime = new ElapsedTime();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front right");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear left");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear right");

        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake = hardwareMap.dcMotor.get("left intake");
        rightIntake = hardwareMap.dcMotor.get("right intake");

        leftLatch = hardwareMap.servo.get("left latch");
        rightLatch = hardwareMap.servo.get("right latch");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double leftPower = Range.clip(gamepad1.left_stick_y - gamepad1.right_stick_x, -1.0, 1.0);
            double rightPower = Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -1.0, 1.0);

            // Send calculated power to wheels
            frontLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            rearLeftDrive.setPower(leftPower);
            rearRightDrive.setPower(rightPower);

            // Set D-Pad for strafing -> not used for Joe 2019-2020
            if (gamepad1.dpad_left) {
                frontLeftDrive.setPower(0.7);
                frontRightDrive.setPower(-0.7);
                rearLeftDrive.setPower(-0.7);
                rearRightDrive.setPower(0.7);
            } else if (gamepad1.dpad_right) {
                frontLeftDrive.setPower(-0.7);
                frontRightDrive.setPower(0.7);
                rearLeftDrive.setPower(0.7);
                rearRightDrive.setPower(-0.7);
            }

            // Map triggers to intake motors
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) { // intake is more power because the brick can be slippery
                leftIntake.setPower(0.5);
                rightIntake.setPower(-0.5);
            } else if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) { // release is less power so it doesn't shoot the brick
                leftIntake.setPower(-0.2);
                rightIntake.setPower(0.2);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }

            // Map bumpers to foundation latches (Servo angles were determined by lab measurements)
            if (gamepad1.left_bumper) {
                leftLatch.setPosition(0.5);
                rightLatch.setPosition(0.3);

                latchStatus = "Latched";
            } else if (gamepad1.right_bumper) {
                leftLatch.setPosition(1);
                rightLatch.setPosition(0);

                latchStatus = "Unlatched";
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", -gamepad1.left_stick_y, -gamepad1.right_stick_y);
            telemetry.addData("Servos", latchStatus);
            telemetry.update();
        }
    }
}