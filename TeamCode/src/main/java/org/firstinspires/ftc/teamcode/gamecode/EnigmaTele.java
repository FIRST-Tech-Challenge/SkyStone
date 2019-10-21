package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Enigma;

/**
 * Created by Aila on 2017-08-21.
 */
public class EnigmaTele extends TeleOpMode {

    private Enigma code;

    @Override
    public void initialize() {
        code = new Enigma();
        code.stops();
    }

    @Override
    public void loopOpMode() {

        code.motorL.setPower(joy1.y1());
        code.motorR.setPower(-joy1.y2());

        if (joy1.leftTrigger()){
            code.collect();
        }
        else{
            code.intake.setPower(0);
        }

        if (joy1.rightTrigger()){
            code.shoot();
        }
        else{
            code.shoot.setPower(0);
        }

        if (joy1.rightBumper()){
            code.feed();
        }
        else{
            code.intake.setPower(0);
        }

    }
}
