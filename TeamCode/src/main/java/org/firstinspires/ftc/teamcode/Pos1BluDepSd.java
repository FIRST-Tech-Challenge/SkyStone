package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PWMOutput;

import java.security.Policy;

@Autonomous

public class Pos1BluDepSd extends MecanumAutoCentral {

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");
        fL.setDirection(DcMotor.Direction.REVERSE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        drive(POWER, 61);
        strafeRight(POWER, 11);
        drive(POWER, -23);
        strafeRight(POWER, 9);
        drive(POWER, -11);
        strafeLeft(POWER, 50);
        strafeRight(POWER, 51);
        drive(POWER, 17);
        strafeRight(POWER, 8);
        drive(POWER, -6);
        strafeRight(POWER, 16);
        drive(POWER, -12);
        strafeLeft(POWER, 75);
    }
}
