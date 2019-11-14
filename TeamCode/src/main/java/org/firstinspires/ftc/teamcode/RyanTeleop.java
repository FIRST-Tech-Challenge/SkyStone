package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RyanTeleop")
public class RyanTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    boolean clamp_open = true;  // TODO UNUSED REMOVE ME

    private void setClamp(boolean open, boolean close) {
        // Formatting matters..  We indent 4 spaces for a very good reason so it's legible.
        // Those grey lines help you find the flow control through the code.
        // Functionally this is great - We need to adjust MID_SERVO -  I would rename this as
        // CLAMP_OPEN_POSITION
            if (open) {
                robot.clamp.setPosition(robot.MID_SERVO);
            }

             if (close) {
            robot.clamp.setPosition(robot.CLAMP_CLOSE_DISTANCE);
        }
    }

    private void moveClampRotator(double clamp_rotator_set) {
        if (clamp_rotator_set > 0 ){
            robot.clampRotator.setPosition(.001 + robot.clampRotator.getPosition());
        } else {
            robot.clampRotator.setPosition(-.001 + robot.clampRotator.getPosition());
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

            if (gamepad2.left_bumper || gamepad2.right_bumper) {
                setClamp(gamepad2.left_bumper, gamepad2.right_bumper);
            }

            if (gamepad2.right_stick_y != 0) {
                moveClampRotator(gamepad2.right_stick_y);
            }
        }
    }
}
