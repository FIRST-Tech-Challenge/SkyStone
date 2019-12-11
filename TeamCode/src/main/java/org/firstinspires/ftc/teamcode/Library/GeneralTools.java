package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;

public class GeneralTools {
    private LinearOpMode opMode;

    public GeneralTools(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * pauses the program for additional seconds
     * @param timeStop double, in Milliseconds
     */
    public void stopForMilliSeconds(double timeStop) {
        double time = System.currentTimeMillis();

        while ((System.currentTimeMillis() < time + timeStop) && !opMode.isStopRequested()) {}
    }

    public static double calculateControllerSmooting(double controllerValue, double smootingFactor) {
        if (controllerValue > 0) {
            return -smootingFactor*Math.exp(Math.log((smootingFactor-1)/smootingFactor)*controllerValue)+smootingFactor;
        } else {
            return -(-smootingFactor*Math.exp(Math.log((smootingFactor-1)/smootingFactor)*-controllerValue)+smootingFactor);
        }

    }
}

