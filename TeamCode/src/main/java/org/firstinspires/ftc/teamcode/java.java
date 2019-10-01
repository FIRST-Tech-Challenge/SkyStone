package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class java extends LinearOpMode {

    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Hello there", 4396);
            telemetry.update();
        }
    }
}
