package org.firstinspires.ftc.teamcode.examples;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.TrackBall;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Robot;

/**
 * Created by Windows on 2016-07-07.
 */
@TeleOp
public class TrackBallTeleOp extends TeleOpMode {

    Robot robot;
    TrackBall mouse;

    @Override
    public void initialize() {
        robot = new Robot();
        mouse = new TrackBall("rightFore", "rightBack");
    }

    @Override
    public void loopOpMode() {
        robot.driveL(joy1.y1());
        robot.driveR(joy1.y2());

        Log.i("EncTiks", mouse.getEncTiks().toString());
    }
}
