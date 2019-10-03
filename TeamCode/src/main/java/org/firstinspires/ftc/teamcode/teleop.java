package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
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
        double FL,FR,BL,BR;      // motor speed variables
        double X1,Y1,X2,Y2;      // joystick position variables
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
            if (egamepad1.right_bumper.pressed)
                speed += 0.1;
            if (egamepad1.right_trigger.pressed)
                speed -= 0.1;
            speed = Drive.limitSpeedPositive(speed);

            // check for move input
            // if dpad pressed, do that move, otherwise use joysticks
            if (egamepad1.dpad_up.state) {
                Drive.moveForward(speed);
            } else if (egamepad1.dpad_down.state) {
                Drive.moveBackward(speed);
            } else if (egamepad1.dpad_left.state) {
                Drive.strafeLeft(speed);
            } else if (egamepad1.dpad_right.state) {
                Drive.strafeRight(speed);
            } else {
                // Reset speed variables
                FL = 0;     FR = 0;     BL = 0;     BR = 0;

                // Get joystick values
                Y1 = -gamepad1.left_stick_y;    // invert so up is positive
                X1 = gamepad1.left_stick_x;
                Y2 = -gamepad1.right_stick_y;   // Y2 is not used at present
                X2 = gamepad1.right_stick_x;

                // Forward/back movement
                // Side to side movement
                // Rotation movement
                FL += Y1;   FR += Y1;   BL += Y1;   BR += Y1;
                FL += X1;   FR -= X1;   BL -= X1;   BR += X1;
                FL += X2;   FR -= X2;   BL += X2;   BR -= X2;

                // scale for speed
                FL *= speed;    FR *= speed;    BL *= speed;    BR *= speed;

                Drive.moveMotors(FL, FR, BL, BR);
            }

            telemetry.addLine("Speed: " + speed);
            telemetry.update();

            // let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }
}
