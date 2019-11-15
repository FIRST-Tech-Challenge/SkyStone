package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.motion.Clamp;

@TeleOp(name = "RyanTeleop")
public class RyanTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    Clamp clamp = new Clamp();

    @Override
    public void runOpMode() {

        // Initialize, wait for start
        robot.init(hardwareMap, telemetry);
        waitForStart();

        // Begins while loop, updates telemetry
        while (opModeIsActive()) {
            telemetry.addData("Status:", "Started");
            telemetry.update();

            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                clamp.setClamp(robot, gamepad2.left_bumper, gamepad2.right_bumper);
            }

            if (gamepad2.right_stick_y != 0) {
                clamp.moveClampRotator(robot, -gamepad2.right_stick_y);
            }
        }
    }
}
