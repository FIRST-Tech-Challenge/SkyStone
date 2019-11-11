

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Manual Driving OpMode", group="Linear Opmode")

public class ManualDriveOpMode extends LinearOpMode {

    ScoopArmBot robot = new ScoopArmBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.driveByVector(gamepad1.left_stick_x, gamepad1.left_stick_y);
            robot.scoopSetPosition(gamepad1.b, gamepad1.y, gamepad1.x);
            robot.scoopFreeRun(gamepad1.right_bumper, gamepad1.left_bumper);

        }
    }
}
