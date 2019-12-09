package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;

public class GeneralTools {
    HardwareMap hardwareMap;
    HardwareChassis robot;

    private LinearOpMode opMode;
    public GeneralTools(LinearOpMode opMode, HardwareChassis robot) {
        this.opMode = opMode;

        this.robot = robot;
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
        robot.servo_grab.setPosition(0.6);
    }

    public void openClamp () {
        robot.servo_2.setPosition(0.1);
    }
}

