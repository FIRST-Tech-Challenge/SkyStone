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
    double time = 2.0;
    double distance = 2.0;
    double angle = 90.0;
    double toangle = 0.0;

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
                    testAngle();
                    break;
                case 3:
                    testToAngle();
                    break;
                case 4:
                    testColor();
                    break;
                default:
                    mode = 0;
                    break;
            }
            telemetry.update();

            // let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }

    public double adjustValue(double val, double inc, double min, double max) {
        if (egamepad1.left_bumper.pressed)
            val += inc;
        if (egamepad1.left_trigger.pressed)
            val -= inc;
        val = Math.max(min, Math.min(val, max));
        return val;
    }

    public void testTime() {
        final double INC_TIME = 0.1;
        final double MIN_TIME = 0.1;
        final double MAX_TIME = 5.0;

        // adjust values
        time = adjustValue(time, INC_TIME, MIN_TIME, MAX_TIME);
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
        final double INC_DISTANCE = 1.0;
        final double MIN_DISTANCE = 10.0;
        final double MAX_DISTANCE = 50.0;

        // adjust values
        distance = adjustValue(distance, INC_DISTANCE, MIN_DISTANCE, MAX_DISTANCE);
        telemetry.addLine("Distance: " + distance);

        // check for move input
        if (egamepad1.dpad_up.released) {
            Drive.moveForwardDistance(speed, distance);
        } else if (egamepad1.dpad_down.released) {
            Drive.moveBackwardDistance(speed, distance);
        } else if (egamepad1.dpad_left.released) {
            Drive.strafeLeftDistance(speed, distance);
        } else if (egamepad1.dpad_right.released) {
            Drive.strafeRightDistance(speed, distance);
        }
    }

    public void testAngle() {
        final double INC_ANGLE = 5.0;
        final double MIN_ANGLE = 10.0;
        final double MAX_ANGLE = 180.0;

        // adjust values
        angle = adjustValue(angle, INC_ANGLE, MIN_ANGLE, MAX_ANGLE);
        telemetry.addLine("Angle: " + angle);

        // check for move input
        if (egamepad1.dpad_left.released) {
            Drive.turnLeftAngle(speed, angle);
        } else if (egamepad1.dpad_right.released) {
            Drive.turnRightAngle(speed, angle);
        }
    }

    public void testToAngle() {
        final double INC_TO_ANGLE = 5.0;
        final double MIN_TO_ANGLE = -180.0;
        final double MAX_TO_ANGLE = 180.0;

        // adjust values
        toangle = adjustValue(toangle, INC_TO_ANGLE, MIN_TO_ANGLE, MAX_TO_ANGLE);
        telemetry.addLine("To Angle: " + toangle);
        telemetry.addLine("Current Angle: " + Drive.IMU.getAngle());

        // check for move input
        if (egamepad1.dpad_left.released) {
            Drive.turnToAngle(speed, toangle);
        } else if (egamepad1.dpad_right.released) {
            Drive.turnToAngle(speed, toangle);
        }
    }

    public void testColor() {

        Drive.Color.Telemetry();

        // check for move input
        if (egamepad1.dpad_up.released) {
            Drive.driveUntilColor(speed);
        }
    }
}
