package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;

public class GeneralTools {
    HardwareChassis robot;
    OmniWheel omniWheel;

    public double ap_underBridgeForward;
    public double ap_forwardGrabStone;
    public double bcap_underBridge;
    public double bcap_passBridge;

    private LinearOpMode opMode;

    public GeneralTools(LinearOpMode opMode, HardwareChassis robot) {
        this.opMode = opMode;
        this.robot = robot;
        omniWheel = new OmniWheel(robot);


        ap_underBridgeForward = 50;
        ap_forwardGrabStone = 55; //65
        bcap_underBridge = 90; // backup program from a2/a5 to under bridge
        bcap_passBridge = 160; //backup program from a2 pass bridge
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
    public void closeClamp () {
        GeneralTools.closeClamp(robot);
    }

    public void openClamp() {
        GeneralTools.openClamp(robot);
    }

    public void grabFoundation() {
        GeneralTools.grabFoundation(robot);
    }

    public void releaseFoundation() {
        GeneralTools.releaseFoundation(robot);
    }

    public static void closeClamp (HardwareChassis robot) {
        robot.servo_grab.setPosition(1);
    }

    public static void openClamp(HardwareChassis robot) {
        robot.servo_grab.setPosition(0.2);
    }


    public static void grabFoundation(HardwareChassis robot) {
        robot.servo_claw_left.setPosition(0.6); // 0.6
        robot.servo_claw_right.setPosition(0.4); // 0.4
    }

    public static void releaseFoundation(HardwareChassis robot) {
        robot.servo_claw_left.setPosition(0.1);
        robot.servo_claw_right.setPosition(0.9);
    }

    public void backTillButtons(HardwareChassis robot) {
        while((robot.touch_right.getState() && robot.touch_left.getState()) || (robot.touch_right.getState() && !robot.touch_left.getState()) || (!robot.touch_right.getState() && robot.touch_left.getState())) {
            omniWheel.setMotors(-0.4, 0, 0);
        }
        omniWheel.setMotors(0, 0, 0);
    }

    public void examplePush() {
        //here is something important written!
    }

}

