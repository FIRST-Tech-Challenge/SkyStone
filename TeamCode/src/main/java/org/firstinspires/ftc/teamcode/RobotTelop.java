package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.motion.Clamp;
import org.firstinspires.ftc.teamcode.motion.Kicker;
import org.firstinspires.ftc.teamcode.motion.DriveTrain;
import org.firstinspires.ftc.teamcode.motion.LeverArm;


@TeleOp(name="RobotTeleop:)", group="Robot")
public class RobotTelop extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot      = new RobotHardware();   // Use a Pushbot's hardware
    LeverArm lever_arm = new LeverArm();
    Clamp clamp = new Clamp();
    DriveTrain tank_drive = new DriveTrain();
    Kicker kicker = new Kicker();

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

            if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
                tank_drive.drive_train(robot, gamepad1.left_stick_y, gamepad1.right_stick_y);
            }
            else {
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
            }

            if (gamepad1.right_trigger > 0 ) {
                kicker.moveKicker(robot,gamepad1.right_trigger);

            } else {
                robot.kicker.setPosition(robot.KICKER_START);
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