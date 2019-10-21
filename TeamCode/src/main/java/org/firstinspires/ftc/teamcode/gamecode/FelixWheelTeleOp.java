package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Felix;

/**
 * Created by Aila on 2017-12-06.
 */
@Disabled
@TeleOp
public class FelixWheelTeleOp extends TeleOpMode {

    private Felix bot;

    boolean reverse = false;

    @Override
    public void initialize() {
        bot = new Felix();
        bot.stop();
    }

    @Override
    public void loopOpMode() {
        double driveLeft = joy2.y1();
        double driveRight = joy2.y2();

        double intakeLeft = joy1.y1();
        double intakeRight = joy1.y2();

        if (joy1.rightTrigger()) {
            bot.glifter.setPower(0.5);
        }
        else{
            bot.glifter.setPower(0);
        }

        if (joy1.leftTrigger()) {
            bot.glifter.setPower(-0.5);
        }
        else{
            bot.glifter.setPower(0);
        }

        if (joy2.rightTrigger()) {
            driveLeft = driveLeft * 0.4;
            driveRight = driveRight * 0.4;
        }

        if (joy2.leftTrigger()) {
            driveLeft = driveLeft * -1;
            driveRight = driveRight * -1;
        }

        bot.driveL(driveLeft);
        bot.driveR(driveRight);

        bot.wheelL.setPower(intakeLeft);
        bot.wheelR.setPower(intakeRight);

        /*
        if (joy2.rightBumper()) {
            bot.glifter.setPower(1);
        } else if (joy2.rightTrigger()) {
            bot.glifter.setPower(-1);
        } else {
            bot.glifter.setPower(0);
        }//else
        */


        if (joy2.buttonX()) {
            bot.jewelL.setPosition(1);
            bot.jewelR.setPosition(0.2);
        }
        if (joy2.buttonB()) {
            bot.jewelL.setPosition(0.2);
            bot.jewelR.setPosition(1);
        }
    }
}
