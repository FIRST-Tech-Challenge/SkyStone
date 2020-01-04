package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Fermion;

/**
 * Created by Windows on 2016-01-30.
 */
@Autonomous
public class TrackballDemo extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        final Fermion up = new Fermion(true);

        waitForStart();
        up.addVeerCheckRunnable();

        up.track(30,1277,0.7);
    }
}
