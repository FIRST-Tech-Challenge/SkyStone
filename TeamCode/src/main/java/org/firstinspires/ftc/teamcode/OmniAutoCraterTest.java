package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Robotics on 9/24/2017.
 */

//@Autonomous(name="Omni: AutoCraterTest", group ="Test")
public class OmniAutoCraterTest extends OmniAutoClassTest {
    @Override
    public void runOpMode() throws InterruptedException {
        int timeout = 0;
        setupRobotParameters(6, 50.9);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        //blue back
        waitForStart();

        Land();

        // Allow tensorflow to adjust itself
        sleep(1000);

        int elementPosition = sampleElements(POSITION_LEFT);
        if(elementPosition == POSITION_UNKNOWN) {
            elementPosition = POSITION_LEFT;
        }

        sampleCrater(elementPosition);

        endAuto();
    }
}
