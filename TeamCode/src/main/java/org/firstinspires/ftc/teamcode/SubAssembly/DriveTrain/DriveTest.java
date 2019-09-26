package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */

// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Drive Test", group = "Test")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // declare local variables
        double speed = 0.3;

        // display welcome message
        telemetry.addLine("Drive Test: ");

        // create extended gamepads (for press and release options)
        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        // create and initialize sub-assemblies
        DriveControl Drive = new DriveControl();
        Drive.init(this);

        // wait for PLAY button to be pressed on driver station
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        waitForStart();

        // loop until STOP button pressed on driver station
        while (opModeIsActive()) {

            // update extended gamepads
            egamepad1.updateEdge();
            egamepad2.updateEdge();

            // check speed input
            if (egamepad1.left_stick_button.pressed) {
                speed += 0.1;
                if (speed > 1.0) speed = 1.0;
            } else if (egamepad1.left_bumper.pressed) {
                speed -= 0.1;
                if (speed < 0) speed = 0;
            }

            // check for move input
            if (egamepad1.dpad_up.state) {
                Drive.moveForward(speed);
            } else if (egamepad1.dpad_down.state) {
                Drive.moveBackward(speed);
            } else if (egamepad1.dpad_left.state) {
                Drive.strafeLeft(speed);
            } else if (egamepad1.dpad_right.state) {
                Drive.strafeRight(speed);
            } else if (gamepad1.left_stick_x > 0.4) {
                Drive.turnRight(speed);
            } else if (gamepad1.left_stick_x < -0.4) {
                Drive.turnLeft(speed);
            } else {
                Drive.stop();
            }

            telemetry.addLine("Speed: " + speed);
            telemetry.update();

            // let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }
}
