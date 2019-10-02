package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */
// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Gamepad Test", group = "Test")
public class GamepadTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // display welcome message
        telemetry.setAutoClear(false);
        telemetry.addLine("GamepadTest: ");
        telemetry.update();

        // create extended gamepads (for press and release options)
        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

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

            telemetry.addLine("Gamepad 1 states");

            if (egamepad1.a.state) telemetry.addLine("a");
            if (egamepad1.b.state) telemetry.addLine("b");
            if (egamepad1.x.state) telemetry.addLine("x");
            if (egamepad1.y.state) telemetry.addLine("y");

            if (egamepad1.left_bumper.state) telemetry.addLine("left_bumper");
            if (egamepad1.left_trigger.state) telemetry.addLine("left_trigger");
            if (egamepad1.left_stick_button.state) telemetry.addLine("left_stick_button");
            if (egamepad1.right_bumper.state) telemetry.addLine("right_bumper");
            if (egamepad1.right_trigger.state) telemetry.addLine("right_trigger");
            if (egamepad1.right_stick_button.state) telemetry.addLine("right_stick_button");

            if (egamepad1.dpad_down.state) {
                telemetry.addLine("dpad_down");
            } else if (egamepad1.dpad_up.state) {
                telemetry.addLine("dpad_up");
            } else if (egamepad1.dpad_left.state) {
                telemetry.addLine("dpad_left");
            } else if (egamepad1.dpad_right.state) {
                telemetry.addLine("dpad_right");
            }

            telemetry.update();

            //let the robot have a little rest, sleep is healthy
            sleep(40);
        }

    }
}
