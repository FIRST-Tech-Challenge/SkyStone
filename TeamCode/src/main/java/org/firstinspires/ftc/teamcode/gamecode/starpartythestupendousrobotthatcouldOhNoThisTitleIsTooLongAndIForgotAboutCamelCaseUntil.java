package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.LinearServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;

/**
 * Created by khoui on 2018-05-24.
 */
@TeleOp
@Disabled
public class starpartythestupendousrobotthatcouldOhNoThisTitleIsTooLongAndIForgotAboutCamelCaseUntil extends TeleOpMode{

    Motor Right = null;
    Motor Left = null;
    Motor Front = null;
    Motor Back = null;
    FXTServo CLAWWW = null;
    LinearServo serbo = null;


    @Override
    public void initialize() {
        Right = new Motor ("driveR");
        Left = new Motor ("driveL");
        Left.setReverse(true);

        Front = new Motor ("driveF");
        Back = new Motor ("driveB");
        Back.setReverse(true);

        CLAWWW = new FXTServo("CLAW");
        serbo = new LinearServo("Serbo");

    }

    @Override
    public void loopOpMode() {
        Right.setPower(joy1.y1());
        Left.setPower(joy1.y1());
        Front.setPower(joy1.x1());
        Back.setPower(joy1.x2());

        serbo.setPosition(joy1.y2());

        if (joy1.rightTrigger()) {
            CLAWWW.setPosition(1);
        } else if (joy1.rightBumper())
            CLAWWW.setPosition(-1);
    }
}
