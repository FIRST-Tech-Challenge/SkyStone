package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;

public class GeneralTools {
    private LinearOpMode opMode;
    public GeneralTools(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    HardwareMap hardwareMap;

    public HardwareChassis hardwareChassis = new HardwareChassis(hardwareMap);


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
        hardwareChassis.servo_grab.setPosition(0.6);
    }

    public void openClamp () {
        hardwareChassis.servo_2.setPosition(0.1);
    }
}

