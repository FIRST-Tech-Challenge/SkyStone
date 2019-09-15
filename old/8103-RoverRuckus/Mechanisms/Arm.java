package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.Utilities.Control.HoldingPIDMotor;

@Config
public class Arm {
    public static double MAX_POWER = 0.9;

    public static int POS_DIFFERENCE = 2100;
    public static double COLLECT_THRESHOLD = 2.35;
    //public static int COLLECT_THRESHOLD = 2500;
    //public static int DEPOSIT_THRESHOLD = 2000;

    public static final BNO055IMU.Parameters metricParameters = new BNO055IMU.Parameters();
    static {
        metricParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        metricParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        metricParameters.magPowerMode = BNO055IMU.MagPowerMode.SLEEP;
    }

    private HoldingPIDMotor leftFlipper, rightFlipper;
    private DcMotorEx extender;

    public Arm (DcMotorEx leftMotor, DcMotorEx rightMotor, DcMotorEx extender) {
        leftFlipper  = new HoldingPIDMotor(leftMotor, MAX_POWER);
        rightFlipper = new HoldingPIDMotor(rightMotor, MAX_POWER);
        this.extender = extender;
    }

    public void setPower(double p) {
        leftFlipper.setPower(p);
        rightFlipper.setPower(p);
    }

    public int getEncoderPosition() {
        return (leftFlipper.getCurrentPosition() +
                rightFlipper.getCurrentPosition()) / 2;
    }

    /*public double getPositionRadians() {
        Acceleration acc = armIMU.getGravity();
        double raw = Math.atan2(acc.zAccel, acc.yAccel);
        if (raw < 0) {
            return -raw;
        } else {
            return Math.PI * 2 - raw;
        }
    }

    public boolean isCollecting() {
        return getPositionRadians() > COLLECT_THRESHOLD;
    }*/

    public void collect() {
        leftFlipper.setTargetPos(leftFlipper.getCurrentPosition() + POS_DIFFERENCE);
        rightFlipper.setTargetPos(rightFlipper.getCurrentPosition() + POS_DIFFERENCE);
    }

    public void deposit() {
        leftFlipper.setTargetPos(leftFlipper.getCurrentPosition() - POS_DIFFERENCE);
        rightFlipper.setTargetPos(rightFlipper.getCurrentPosition() - POS_DIFFERENCE);
    }



}
