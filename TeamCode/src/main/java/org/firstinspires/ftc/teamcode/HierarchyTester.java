package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "Hierarchy Tester", group = "beta")
public class HierarchyTester extends LinearOpMode {
    private Trobot trobot;

    private boolean latched = false;
    private String servoStatus = "Pending";

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        trobot = new Trobot(hardwareMap);

        waitForStart();
        trobot.runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            double leftPower = Range.clip(gamepad1.left_stick_y - gamepad1.right_stick_x, -1.0, 1.0);
            double rightPower = Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -1.0, 1.0);

            trobot.drivetrain.drive(leftPower, rightPower);

            // Map D-Pad to strafing
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
                latched = true;
            } else if (gamepad1.right_bumper) {
                trobot.component.latch("UNLATCH");
                latched = false;
            }

            if (latched) {
                servoStatus = "latched";
            } else {
                servoStatus = "unlatched";
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + trobot.runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", -gamepad1.left_stick_y, -gamepad1.right_stick_y);
            telemetry.addData("Servos", servoStatus);
            telemetry.update();
        }
    }
}