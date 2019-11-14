package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="RyanTeleop")
public class RyanTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    boolean clamp_open = true;

    private void setClamp(boolean open, boolean close) {
            if (open) {
                robot.clamp.setPosition(robot.MID_SERVO);
            }

             if (close) {
            robot.clamp.setPosition(robot.CLAMP_CLOSE_DISTANCE);
        }
    }

    private void moveClampRotator(double clamp_rotator_set) {

        robot.clampRotator.setPosition((.0001*clamp_rotator_set) + robot.clampRotator.getPosition());
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

            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                setClamp(gamepad2.left_bumper, gamepad2.right_bumper);
            }

            if (gamepad2.right_stick_y != 0) {
                moveClampRotator(gamepad2.right_stick_y);
            }
        }
    }
}
