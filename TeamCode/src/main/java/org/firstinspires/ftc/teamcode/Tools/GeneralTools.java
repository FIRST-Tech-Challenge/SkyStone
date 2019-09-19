package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

}

