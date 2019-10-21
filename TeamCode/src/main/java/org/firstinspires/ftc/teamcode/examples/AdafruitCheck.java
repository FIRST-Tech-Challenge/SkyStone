package org.firstinspires.ftc.teamcode.examples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Robot;

/**
 * Created by Owner on 8/31/2015.
 */
@Autonomous
public class AdafruitCheck extends AutoOpMode{


    @Override
    public void runOp() throws InterruptedException {
        Robot r = new Robot();

        waitForStart();

        r.imuTurnL(90, 0.5);

        sleep(500);
        r.stop();
    }
}