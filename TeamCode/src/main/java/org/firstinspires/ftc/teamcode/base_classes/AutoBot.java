package org.firstinspires.ftc.teamcode.base_classes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.algorithms.VuforiaAutoNav;

/**
 * This class is the class for autonomous robots
 */
public class AutoBot extends Robot {

    //stores the value of sin(45°), or sin(pi/4)
    public double sin45 = Math.sqrt(2) / 2;
    public VuforiaAutoNav nav;
    public float robotWidth = 18; // robot width in inches
    public float robotLength = 18; // robot length in inches

    public float getRobotWidth() {
        return robotWidth;
    }

    public void setRobotWidth(float robotWidth) {
        this.robotWidth = robotWidth;
    }

    public float getRobotLength() {
        return robotLength;
    }

    public void setRobotLength(float robotLength) {
        this.robotLength = robotLength;
    }

    /**
     * constructor for auto bot
     *
     * @param opMode
     */
    public AutoBot(OpMode opMode) {
        this.opMode = opMode;
    }

    /**
     * gets the motor powers
     *
     * @return motor powers
     */
    public double[] getPowers() {
        return new double[]{frontLeft.getPower(), frontRight.getPower(), backRight.getPower(), backLeft.getPower()};
    }

    public VuforiaAutoNav getNav() {
        return nav;
    }

    public void setNav(VuforiaAutoNav nav) {
        this.nav = nav;
    }

    public void initTracking() {
        this.nav = new VuforiaAutoNav(this.opMode.hardwareMap);
        this.nav.initView();
    }

}
