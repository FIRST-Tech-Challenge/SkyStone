package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Felix;

/**
 * Created by Aila on 2018-01-19.
 */
@Disabled
@TeleOp
public class FelixTeloOpL2 extends TeleOpMode{
    private Felix iron;

    boolean reverse = false;

    @Override
    public void initialize() {
        iron = new Felix();
        iron.stop();
    }

    @Override
    public void loopOpMode() {
        double driveLeft = -joy1.y1();
        double driveRight = -joy1.y2();

        double intakeLeft = joy2.y1();
        double intakeRight = joy2.y2();

        if (joy1.rightTrigger()) {
            driveLeft = driveLeft * 0.4;
            driveRight = driveRight * 0.4;
        }

        if (joy1.leftTrigger()) {
            driveLeft = driveLeft * -1;
            driveRight = driveRight * -1;
        }

        while (joy2.rightTrigger()){
            iron.glifter.setPower(0.5);
        }
        iron.glifter.setPower(0);
        while (joy2.leftTrigger()){
            iron.glifter.setPower(-0.5);
        }
        iron.glifter.setPower(0);

        iron.driveL(driveLeft);
        iron.driveR(driveRight);

        iron.wheelL.setPower(intakeLeft);
        iron.wheelR.setPower(intakeRight);


        if (joy2.buttonX()) {
            iron.jewelL.setPosition(0);
            iron.jewelR.setPosition(0);
        }
        if (joy2.buttonB()) {
            iron.jewelL.setPosition(1);
            iron.jewelR.setPosition(1);
        }
    }
}
