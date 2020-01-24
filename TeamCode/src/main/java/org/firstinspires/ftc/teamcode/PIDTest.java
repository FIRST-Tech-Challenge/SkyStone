package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="PID Straight Test", group="Exercises")

public class PIDTest extends LinearOpMode {
    GyroBot robot = new GyroBot(this);


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.resetAngle();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                robot.goBacktoStartAngle();
                sleep(1000);
                robot.getDeltaAngle();
            }
            else if (gamepad1.b) {
                robot.goBacktoStartAnglePID();
                sleep(1000);
                robot.getDeltaAngle();
            }
            else if (gamepad1.dpad_up) {
                robot.driveStraightByGyro(FourWheelsDriveBot.DIRECTION_FORWARD, 1000, 0.8);
            }
            else if (gamepad1.dpad_down) {
                robot.driveStraightByGyro(FourWheelsDriveBot.DIRECTION_BACKWARD, 1000, 0.8);
            }
            else if (gamepad1.dpad_left) {
                robot.driveStraightByGyro(FourWheelsDriveBot.DIRECTION_LEFT, 1000, 0.5);
            }
            else if (gamepad1.dpad_right) {
                robot.driveStraightByGyro(FourWheelsDriveBot.DIRECTION_RIGHT, 1000, 0.5);
            }
        }




    }
}
