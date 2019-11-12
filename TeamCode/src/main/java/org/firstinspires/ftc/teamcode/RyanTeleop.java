package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="RyanTeleop")
public class RyanTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    //
    public void moveClamp(double distance) {

        double clamp_set = (.004 * gamepad2.right_stick_y);
        robot.clamp.setPosition(robot.clamp.getPosition() + clamp_set);
    }
    // Ryan's logic

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status:", "Started");
            telemetry.update();

            //
            if (gamepad2.right_stick_y != 0) {
                moveClamp(gamepad2.right_stick_y);
            }
            // Ryan's code (under "while opModeIsActive")
        }
    }
}