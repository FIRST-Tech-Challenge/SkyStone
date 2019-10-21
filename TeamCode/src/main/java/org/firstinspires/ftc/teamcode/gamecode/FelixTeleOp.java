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

public class FelixTeleOp extends TeleOpMode {

    private Felix bot;

    boolean reverse = false;

    @Override
    public void initialize() {
        bot = new Felix();
        bot.init(hardwareMap);
        bot.stop();
    }

    @Override
    public void loopOpMode() {
        double driveLeft = joy1.y1();
        double driveRight = joy1.y2();

        double glifter = -gamepad2.right_stick_y;
        double hands = gamepad2.left_trigger;

        if (joy1.rightTrigger()) {
            driveLeft = driveLeft * 0.4;
            driveRight = driveRight * 0.4;
        }

        if (joy1.leftTrigger()) {
            driveLeft = driveLeft * -1;
            driveRight = driveRight * -1;
        }

        bot.driveL(driveLeft);
        bot.driveR(driveRight);

        bot.glifter.setPower(glifter);

        if (hands > 0.1) {
            /*
            bot.handL.setPower(0.7);
            bot.handR.setPower(-0.7);
            */
        }
        else {
            /*
            bot.handL.setPower(-0.9);
            bot.handR.setPower(0.9);
            */
        }

        if (joy2.y1() > 0.1) {
            bot.jewelL.setPosition(1);
        }
        if (joy2.y1() < -0.1) {
            bot.jewelL.setPosition(0.2);
        }
    }
}
