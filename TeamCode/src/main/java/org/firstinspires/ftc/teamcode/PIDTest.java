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
            }
        }




    }
}
