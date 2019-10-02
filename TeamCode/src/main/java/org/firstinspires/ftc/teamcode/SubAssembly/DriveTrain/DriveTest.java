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
        telemetry.setAutoClear(false);
        telemetry.addLine("Drive Test: ");
        telemetry.update();

        // create extended gamepads (for press and release options)
        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        // create and initialize sub-assemblies
        DriveControl Drive = new DriveControl();
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

            // check speed input
            if (egamepad1.left_trigger.pressed) {
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

            if (egamepad1.a.state) {
                Drive.moveForwardTime(speed, 3.0);
            }
            if (egamepad1.b.state) {
                Drive.moveForwardDistance(speed, 100);
            }
            if (egamepad1.x.state) {
                telemetry.addLine("Time Test...");
                telemetry.update();
                Drive.moveForwardTime(speed, 3.0);
                Drive.moveBackwardTime(speed, 3.0);
                Drive.strafeLeftTime(speed, 3.0);
                Drive.strafeRightTime(speed, 3.0);
                Drive.turnLeftTime(speed, 3.0);
                Drive.turnRightTime(speed, 3.0);
            }
            if (egamepad1.y.state) {
                telemetry.addLine("Distance Test...");
                telemetry.update();
                Drive.moveForwardDistance(speed, 100);
                Drive.moveBackwardDistance(speed, 100);
                Drive.strafeLeftDistance(speed, 100);
                Drive.strafeRightDistance(speed, 100);
                Drive.turnLeftDistance(speed, 100);
                Drive.turnRightDistance(speed, 100);
            }

            telemetry.addLine("Speed: " + speed);
            telemetry.update();

            // let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }
}
