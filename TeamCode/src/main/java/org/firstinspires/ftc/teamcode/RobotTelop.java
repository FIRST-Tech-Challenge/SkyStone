package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Basic Example", group="Robot")
public class RobotTelop extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot      = new RobotHardware();   // Use a Pushbot's hardware

    private void moveRobot(float x_direction, float y_direction) {
        // Do something
    }

    private void moveLeverArm(float distance) {
        // Do something
    }

    private void moveClampRotator(float distance) {
        // Do something
    }

    private void setClamp(boolean open, boolean close) {
        // Do something
    }


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_stick_x > 0.05 || gamepad1.left_stick_y < 0.05 ) {
                moveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            }

            if (gamepad2.left_stick_y != 0 ) {
                moveLeverArm(gamepad2.left_stick_y);
            }

            if (gamepad2.right_stick_y != 0 ) {
                moveClampRotator(gamepad2.right_stick_y);
            }

            if (gamepad2.left_bumper || gamepad1.right_bumper ) {
                setClamp(gamepad2.left_bumper, gamepad2.right_bumper);
            }

        }
    }
}