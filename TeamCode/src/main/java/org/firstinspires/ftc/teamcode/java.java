package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class java extends LinearOpMode {

    DcMotor hello;

    public void runOpMode() {

        hello = hardwareMap.dcMotor.get("Hello");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}
