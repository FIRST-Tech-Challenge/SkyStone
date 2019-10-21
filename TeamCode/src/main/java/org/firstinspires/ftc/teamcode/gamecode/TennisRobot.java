package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;

/**
 * Created by Alec Krawciw on 2017-05-20.
 */

public class TennisRobot extends TeleOpMode {

    //Declare the motors.
    //This was changed to two motors when we changed to power
    //splitters
    Motor right;
    Motor left;
    Motor shoot;
    Motor collect;
    boolean on = false;

    FXTCRServo feed;


    @Override
    public void initialize() {
        right = new Motor("driveL");
        left = new Motor("driveR");
        left.setReverse(true);

        shoot = new Motor("shoot");
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot.getM().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot.setReverse(true);

        collect = new Motor("collect");

        feed = new FXTCRServo("feed");
    }

    @Override
    public void loopOpMode() {
        right.setPower(joy1.y1());
        left.setPower(joy1.y2());

        if (joy2.leftTrigger()) {
            shoot.setPower(1);
            clearTimer();
            on = true;
        } else if (on && getMilliSeconds() < 1000) {
            shoot.setPower(0.5);
        } else if (on && getMilliSeconds() < 2000) {
            shoot.setPower(0.25);
        } else if (on) {
            shoot.setPower(0);
        }

        RC.t.addData("Power", shoot.getPower());

//        if (joy2.leftTrigger()) {
//            shoot.setPower(1);
//            clearTimer();
//            on = true;
//        } else if(getMilliSeconds() < 5000 && on){
//            shoot.setPower(1 - getMilliSeconds() / 5000.0);
//            RC.t.addData("power", 1 - getMilliSeconds() / 5000.0);
//        } else {
//            shoot.stop();
//            on = false;
//        }

        if (joy2.rightTrigger()) {
            feed.setPower(1);
        } else {
            feed.setPower(0.06);
        }

        if (joy1.rightTrigger()) {
            collect.setPower(1);
        } else if(joy1.rightBumper()){
            collect.setPower(-1);
        } else {
            collect.stop();
        }

    }
}
