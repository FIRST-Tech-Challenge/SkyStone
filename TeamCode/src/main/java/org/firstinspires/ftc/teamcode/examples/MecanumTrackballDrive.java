package org.firstinspires.ftc.teamcode.examples;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.TrackBall;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Fermion;

/**
 * Created by FIYIT on 16-10-14.
 */
@Autonomous
public class MecanumTrackballDrive extends AutoOpMode {

    Fermion rbt;



    @Override
    public void runOp() throws InterruptedException {
        rbt = new Fermion(true);

        waitForStart();
        rbt.resetTargetAngle();

        double tiks = (300 / (1.4 * 25.4 * 3 * Math.PI)) * 1440;
        double speed = 0.7;
        double minSpeed = 0.1;
        double degrees = 0;

        rbt.strafe(degrees, speed, true);

        TrackBall.Point begin = rbt.mouse.getEncTiks();
        TrackBall.Point end = begin.add(new TrackBall.Point(Math.sin(degrees) * tiks, Math.cos(degrees) * tiks));

        Log.i("Points", end + ", " + begin);

        while (opModeIsActive()) {
            TrackBall.Point remaining = end.subtract(rbt.mouse.getEncTiks());

            Log.i("Point", remaining + "");
            if (remaining.hypot() < 100) {
                break;
            }//if

            rbt.strafe(remaining.acot(), Math.abs(speed - minSpeed) * Math.abs(remaining.hypot() / tiks) + minSpeed, true);
            rbt.veerCheck(false);

        }

        rbt.stop();

    }
}
