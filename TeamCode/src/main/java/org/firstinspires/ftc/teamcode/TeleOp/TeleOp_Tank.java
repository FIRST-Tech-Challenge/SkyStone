package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SourceFiles.Trobot;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a four wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Tank Mode", group = "Tank Mode")
public class TeleOp_Tank extends LinearOpMode {
    public Trobot trobot;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        trobot = new Trobot(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        trobot.runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Tank Mode uses both sticks to control each side.
            double leftPower = gamepad1.left_stick_y;
            double rightPower = gamepad1.right_stick_y;

            trobot.drivetrain.drive(leftPower, rightPower);

            // Set D-Pad for strafing -> not used for Joe 2019-2020
            if (gamepad1.dpad_left) {
                trobot.drivetrain.strafe("LEFT");
            } else if (gamepad1.dpad_right) {
                trobot.drivetrain.strafe("RIGHT");
            }

            // Map triggers to intake motors
            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {
                trobot.component.intake("INTAKE");
            } else if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {
                trobot.component.intake("RELEASE");
            } else {
                trobot.component.intake("STOP");
            }

            // Map bumpers to foundation latches
            if (gamepad1.left_bumper) {
                trobot.component.latch("LATCH");
            } else if (gamepad1.right_bumper) {
                trobot.component.latch("UNLATCH");
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + trobot.runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", -gamepad1.left_stick_y, -gamepad1.right_stick_y);
            telemetry.addData("Servos", trobot.component.latchStatus);
            telemetry.update();
        }
    }
}