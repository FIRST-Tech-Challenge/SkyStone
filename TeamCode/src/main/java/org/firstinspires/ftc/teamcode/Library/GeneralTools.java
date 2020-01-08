package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;

public class GeneralTools {
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

    public static double calculateControllerSmooting(double controllerValue, double smootingFactor) {
        if (controllerValue > 0) {
            return -smootingFactor*Math.exp(Math.log((smootingFactor-1)/smootingFactor)*controllerValue)+smootingFactor;
        } else {
            return -(-smootingFactor*Math.exp(Math.log((smootingFactor-1)/smootingFactor)*-controllerValue)+smootingFactor);
        }

    }

    /**
     * set claw to close
     */
    public void openClamp () {
        robot.servo_grab.setPosition(0.5);
    }

    public void closeClamp() {
        robot.servo_grab.setPosition(1);
    }

    public void grabFoundation() {
        robot.servo_claw_left.setPosition(0.1);
        robot.servo_claw_right.setPosition(0.9);
    }

    public void releaseFoundation() {
        robot.servo_claw_left.setPosition(0.5);
        robot.servo_claw_right.setPosition(0.5);
    }
}

