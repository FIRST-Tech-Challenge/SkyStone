package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;
import org.firstinspires.ftc.teamcode.robots.Robot;

@TeleOp
public class SummerBot extends TeleOpMode {

    Motor lift;
    Motor driveR;
    Motor driveL;

    @Override
    public void initialize() {
        lift = new Motor("lift");

    }

    @Override
    public void loopOpMode() {
        if (joy1.buttonY() == true) {
            lift.setPower(1);
        }
        else if (joy1.buttonA()== true) {
            lift.setPower(-1);
        }
        else {
            lift.setPower(0);
        }
        driveL.setPower(joy1.y1());
        driveR.setPower(joy1.y2());
    }
}
