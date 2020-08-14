package org.firstinspires.ftc.teamcode.FTC2021;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "MainTeleOp", group = "Linear Opmode")
public class MainTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap,telemetry,this);

        ThreadLoop t = new ThreadLoop(robot,robot.linearOpMode);
        robot.hardwareCollection.fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        t.run();

        waitForStart();

        while (opModeIsActive()) {
            robot.telemetry.addLine(Integer.toString(robot.hardwareCollection.fLeft.getCurrentPosition()));
        }

    }
}
