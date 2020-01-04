package org.firstinspires.ftc.teamcode.examples;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Fermion;

/**
 * Created by Windows on 2017-03-04.
 */
@Autonomous
public class VeerCheckDemo extends AutoOpMode{


    @Override
    public void runOp() throws InterruptedException {
        Fermion f = new Fermion(true);
        Log.i("VeerProportional", RC.globalDouble("VeerProportional") + "");
        Log.i("VeerIntegral", RC.globalDouble("VeerIntegral") + "");
        Log.i("VeerDerivative", RC.globalDouble("VeerDerivative") + "");

        f.resetTargetAngle();
        f.addVeerCheckRunnable();

        while (opModeIsActive()) {
            idle();
        }//while
    }
}