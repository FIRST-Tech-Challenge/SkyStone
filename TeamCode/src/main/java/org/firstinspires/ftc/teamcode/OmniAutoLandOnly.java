package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Robotics on 9/24/2017.
 */

//@Autonomous(name="Omni: AutoLandOnly", group ="Competition")
public class OmniAutoLandOnly extends OmniAutoClass {
    @Override
    public void runOpMode() throws InterruptedException {
        int timeout = 0;
        setupRobotParameters(6, 50.9);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        //blue back
        waitForStart();


        endAuto();
    }
}
