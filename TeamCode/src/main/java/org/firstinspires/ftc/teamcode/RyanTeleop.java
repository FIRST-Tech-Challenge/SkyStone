package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="RyanTeleop")
public class RyanTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    boolean clamp_open = true;
    boolean open = gamepad2.left_bumper;
    boolean close = gamepad2.right_bumper;

    private void setClamp(boolean open, boolean close) {

        if (!clamp_open) {
            if (gamepad2.left_bumper) {
                robot.clamp.setPosition(robot.MID_SERVO);
                clamp_open = true;
            }
        }

        if (clamp_open) {
            if (gamepad2.right_bumper) {
                robot.clamp.setPosition(robot.CLAMP_CLOSE_DISTANCE);
                clamp_open = false;
            }
        }
    }

    @Override
    public void runOpMode() {

        // Initialize, wait for start
        robot.init(hardwareMap, telemetry);
        waitForStart();

        // Begins while loop, updates telemetry
        while (opModeIsActive()) {
            telemetry.addData("Status:", "Started");
            telemetry.update();

            if (gamepad2.left_bumper || gamepad1.right_bumper) {
                setClamp(gamepad2.left_bumper, gamepad2.right_bumper);
            }
        }
    }
}
