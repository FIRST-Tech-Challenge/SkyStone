package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="RyanTeleop")
public class RyanTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    //// Takes a distance and adds/subtracts with the previous position
    public void moveClamp(double distance) {

        double clamp_set = (.004*distance);
        robot.clamp.setPosition(robot.clamp.getPosition() + clamp_set);
    }
    //// Ryan's logic

    @Override
    public void runOpMode() {

        // Initialize, wait for start
        robot.init(hardwareMap, telemetry);
        waitForStart();

        //// Begins while loop, updates telemetry
        while (opModeIsActive()) {
            telemetry.addData("Status:", "Started");
            telemetry.update();

            // inputs the stick's y value into moveClamp
            if (gamepad2.right_stick_y != 0) {
                moveClamp(gamepad2.right_stick_y);
            }
            //// Ryan's code (under "while opModeIsActive")
        }
    }
}