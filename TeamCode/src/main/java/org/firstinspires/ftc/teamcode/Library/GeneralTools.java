package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    /**
     * set claw to close
     */
    public void grabSkysstone () {
        servo_1.setPosition(0.6);
        servo_1.setPosition(0.1);
    }
}

