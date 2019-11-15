package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.motion.Clamp;
import org.firstinspires.ftc.teamcode.motion.LeverArm;


@TeleOp(name="RobotTeleop:)", group="Robot")
public class RobotTelop extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot      = new RobotHardware();   // Use a Pushbot's hardware
    LeverArm lever_arm = new LeverArm();
    Clamp clamp = new Clamp();

    private void moveRobot(float x_direction, float y_direction) {
        // Do something
    }

    private void moveKicker(float distance) {
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

            if (gamepad1.right_trigger > 0 ) {
                moveKicker(gamepad1.right_trigger);
            }

            if (gamepad2.left_stick_y < .5 && gamepad2.left_stick_y > -.5) {
                lever_arm.leverArmStay(robot);
            }
            if (gamepad2.left_stick_y > .5 || gamepad2.left_stick_y < -.5) {
                lever_arm.moveLeverArm(robot, telemetry, -gamepad2.left_stick_y);
            }

            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                clamp.setClamp(robot, gamepad2.left_bumper, gamepad2.right_bumper);
            }

            if (gamepad2.right_stick_y != 0) {
                clamp.moveClampRotator(robot, -gamepad2.right_stick_y);
            }


        }
    }
}