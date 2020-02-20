package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */

// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Drive Time", group = "Drive Test")
public class DriveTestTime extends LinearOpMode {

    // declare class variables
    DriveControl Drive = new DriveControl();
    final double INCREMENT_SPEED = 0.1;
    double speed = 0.3;
    double time = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // declare local variables
        int mode = 0;

        // display welcome message
        telemetry.setAutoClear(false);
        telemetry.addLine("Drive Time Test: ");
        telemetry.update();

        // create extended gamepads (for press and release options)
        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        // create and initialize sub-assemblies
        Drive.init(this);

        // wait for PLAY button to be pressed on driver station
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        // loop until STOP button pressed on driver station
        while (opModeIsActive()) {

            // update extended gamepads
            egamepad1.updateEdge();
            egamepad2.updateEdge();

            telemetry.addLine("Press DPAD to drive for time");
            telemetry.addLine("Press Leftstick-x to turn for time");

            // check speed input
            if (egamepad1.right_bumper.pressed)
                speed += INCREMENT_SPEED;
            if (egamepad1.right_trigger.pressed)
                speed -= INCREMENT_SPEED;
            speed = Drive.limitSpeedPositive(speed);
            telemetry.addLine("Speed: " + speed);

            // check time input
            final double INC = 0.1;
            final double MIN = 0.1;
            final double MAX = 5.0;
            if (egamepad1.left_bumper.pressed)
                time += INC;
            if (egamepad1.left_trigger.pressed)
                time -= INC;
            time = Math.max(MIN, Math.min(time, MAX));
            telemetry.addLine("Time: " + time);

            // check for move input
            if (egamepad1.dpad_up.released) {
                Drive.moveForwardTime(speed, time);
            } else if (egamepad1.dpad_down.released) {
                Drive.moveBackwardTime(speed, time);
            } else if (egamepad1.dpad_left.released) {
                Drive.strafeLeftTime(speed, time);
            } else if (egamepad1.dpad_right.released) {
                Drive.strafeRightTime(speed, time);
            } else if (gamepad1.left_stick_x > 0.1) {
                Drive.turnRightTime(speed, time);
            } else if (gamepad1.left_stick_x < -0.1) {
                Drive.turnLeftTime(speed, time);
            }

            telemetry.update();

            // let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }
}
