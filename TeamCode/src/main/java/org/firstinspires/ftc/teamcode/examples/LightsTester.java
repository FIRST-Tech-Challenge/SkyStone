package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.newhardware.Lights;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

/**
 * Created by Windows on 2017-03-31.
 */
@Autonomous
public class LightsTester extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        Lights lights = new Lights("lights");

        waitForStart();

        lights.setLightsState(-1000);

        sleep(5000);

        lights.setLightsState(0);

        sleep(5000);

        lights.flashState(500);

        sleep(2000);
    }//runOp

}
