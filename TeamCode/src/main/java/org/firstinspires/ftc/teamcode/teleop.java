package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Grabber.GrabberControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Lift.LiftControl;
import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */
// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "teleop", group = "Drive")
public class teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // declare local variables
        double X1, Y1, X2, Y2, FL, FR, BL, BR;  // for joystick control
        double speed = 0.3;

        // display welcome message
        telemetry.setAutoClear(false);
        telemetry.addLine("TeleOp");
        telemetry.update();

        // create extended gamepads (for press and release options)
        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        // create and initialize sub-assemblies
        DriveControl Drive = new DriveControl();
        GrabberControl Grabber = new GrabberControl();
        LiftControl Lift = new LiftControl();
        Drive.init(this);
        Grabber.init(this);
        Lift.init(this);

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

            //ready player 1-----------------------------------------------

            // check speed input
            if (egamepad1.right_bumper.pressed)
                speed += 0.1;
            if (egamepad1.right_trigger.pressed)
                speed -= 0.1;
            speed = Drive.limitSpeedPositive(speed);

            // check for move input
            if (egamepad1.dpad_up.state) {
                Drive.moveForward(speed);
            } else if (egamepad1.dpad_down.state) {
                Drive.moveBackward(speed);
            } else if (egamepad1.dpad_left.state) {
                Drive.strafeLeft(speed);
            } else if (egamepad1.dpad_right.state) {
                Drive.strafeRight(speed);
            } else {
                // Get joystick values
                Y1 = -gamepad1.left_stick_y;    // invert so up is positive
                X1 = gamepad1.left_stick_x;
                Y2 = -gamepad1.right_stick_y;   // Y2 is not used at present
                X2 = gamepad1.right_stick_x;

                // Combine Forward/back, Side to side, Rotation movement
                // scale for speed
                FL = (Y1 + X1 + X2) * speed;
                FR = (Y1 - X1 - X2) * speed;
                BL = (Y1 - X1 + X2) * speed;
                BR = (Y1 + X1 - X2) * speed;

                Drive.moveMotors(FL, FR, BL, BR);
            }

            //go player 2-----------------------------------------------

            if (egamepad2.a.released) {
                Grabber.open();
            } else if (egamepad2.b.released) {
                Grabber.close();
            }
            if (egamepad2.right_bumper.released) {
                Grabber.wrist();
            }
            if (egamepad2.x.released) {
                Grabber.extend();
            }
            if (egamepad2.y.released) {
                Grabber.home();
            }

            if (egamepad2.dpad_up.state && !Lift.isLimitTop()) {
                Lift.MoveUp();
            } else if (egamepad2.dpad_down.state && !Lift.isLimitBottom()) {
                Lift.MoveDown();
            } else {
                Lift.Stop();
            }

            telemetry.addLine("Speed: " + speed);
            telemetry.update();

            // let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }
}
