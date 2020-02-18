package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */

// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Drive Auto Test", group = "Test")
public class DriveAutoTest extends LinearOpMode {

    // declare class variables
    GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
    GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);
    DriveControl Drive = new DriveControl();
    final double INCREMENT_SPEED = 0.1;
    double speed = 0.3;
    final double INCREMENT_TIME = 0.1;
    final double MIN_VALUE_TIME = 0.1;
    final double MAX_VALUE_TIME = 5.0;
    double time = 2.0;
    final double INCREMENT_DISTANCE = 1.0;
    final double MIN_VALUE_DISTANCE = 10.0;
    final double MAX_VALUE_DISTANCE = 50.0;
    double distance = 2.0;
    final double INCREMENT_ANGLE = 5.0;
    final double MIN_VALUE_ANGLE = 10.0;
    final double MAX_VALUE_ANGLE = 180.0;
    double angle = 90.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // declare local variables
        int mode = 0;

        // display welcome message
        telemetry.setAutoClear(false);
        telemetry.addLine("Drive Auto Test: ");
        telemetry.update();

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

            // check speed input
            if (egamepad1.right_bumper.pressed)
                speed += INCREMENT_SPEED;
            if (egamepad1.right_trigger.pressed)
                speed -= INCREMENT_SPEED;
            speed = Drive.limitSpeedPositive(speed);
            telemetry.addLine("Speed: " + speed);

            if (egamepad1.a.released)
                mode += 1;
            switch (mode) {
                case 0:
                    testTime();
                    break;
                case 1:
                    testDistance();
                    break;
                case 2:
                    mode = 0;
                    break;
            }
            telemetry.update();

            // let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }

    public void testTime() {
        // check time input
        if (egamepad1.left_bumper.pressed)
            time += INCREMENT_TIME;
        if (egamepad1.left_trigger.pressed)
            time -= INCREMENT_TIME;
        time = Math.max(MIN_VALUE_TIME, Math.min(time, MAX_VALUE_TIME));
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
        } else if (gamepad1.left_stick_x < -0.1) {
            Drive.turnRightTime(speed, time);
        } else if (gamepad1.left_stick_x > 0.1) {
            Drive.turnLeftTime(speed, time);
        }
    }

    public void testDistance() {
        // check time input
        if (egamepad1.left_bumper.pressed)
            distance += INCREMENT_DISTANCE;
        if (egamepad1.left_trigger.pressed)
            distance -= INCREMENT_DISTANCE;
        distance = Math.max(MIN_VALUE_DISTANCE, Math.min(time, MAX_VALUE_DISTANCE));
        telemetry.addLine("Distance: " + distance);
        if (egamepad1.x.pressed)
            angle += INCREMENT_ANGLE;
        if (egamepad1.y.pressed)
            angle -= INCREMENT_ANGLE;
        angle = Math.max(MIN_VALUE_ANGLE, Math.min(time, MAX_VALUE_ANGLE));
        telemetry.addLine("Angle: " + angle);

        // check for move input
        if (egamepad1.dpad_up.released) {
            Drive.moveForwardDistance(speed, distance);
        } else if (egamepad1.dpad_down.released) {
            Drive.moveBackwardDistance(speed, distance);
        } else if (egamepad1.dpad_left.released) {
            Drive.strafeLeftDistance(speed, distance);
        } else if (egamepad1.dpad_right.released) {
            Drive.strafeRightDistance(speed, distance);
        } else if (gamepad1.left_stick_x < -0.1) {
            Drive.turnRightAngle(speed, angle);
        } else if (gamepad1.left_stick_x > 0.1) {
            Drive.turnLeftAngle(speed, angle);
        }
    }
}
