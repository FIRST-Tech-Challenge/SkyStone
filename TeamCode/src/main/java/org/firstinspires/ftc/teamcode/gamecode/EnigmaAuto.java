package org.firstinspires.ftc.teamcode.gamecode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Enigma;

/**
 * Created by Aila on 2017-08-21.
 */


public class EnigmaAuto extends AutoOpMode {
    @Override
    public void runOp() throws InterruptedException {

        final Enigma encrypt = new Enigma();


        waitForStart();
        Log.i("Position", "" + encrypt.motorR.getBaseCurrentPosition());
        encrypt.forwardPosition(75, 400);
        Log.i("Position", String.valueOf(encrypt.motorR.getBaseCurrentPosition()));
        encrypt.feed();
        encrypt.shoot();
        encrypt.reversePosition(75, 400);
        Log.i("Position", String.valueOf(encrypt.motorR.getBaseCurrentPosition()));
        encrypt.imuTurnL(50, 90);
        Log.i("Message", "I'm finished");

    }
}
