package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PWMOutput;

import java.security.Policy;

@Autonomous

public class Pos1BluDepSd extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumAutoCentral.fL = hardwareMap.dcMotor.get("fL");
        MecanumAutoCentral.fR = hardwareMap.dcMotor.get("fR");
        MecanumAutoCentral.bL = hardwareMap.dcMotor.get("bL");
        MecanumAutoCentral.bR = hardwareMap.dcMotor.get("bR");
        MecanumAutoCentral.fL.setDirection(DcMotor.Direction.REVERSE);
        MecanumAutoCentral.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MecanumAutoCentral.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MecanumAutoCentral.bL.setDirection(DcMotor.Direction.REVERSE);
        MecanumAutoCentral.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MecanumAutoCentral.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double POWER = MecanumAutoCentral.POWER;

        waitForStart();

        MecanumAutoCentral.forward(POWER, 61);
        MecanumAutoCentral.strafeRight(POWER, 11);
        MecanumAutoCentral.backward(POWER, 23);
        MecanumAutoCentral.strafeRight(POWER, 9);
        MecanumAutoCentral.backward(POWER, 11);
        MecanumAutoCentral.strafeLeft(POWER, 50);
        MecanumAutoCentral.strafeRight(POWER, 51);
        MecanumAutoCentral.forward(POWER, 17);
        MecanumAutoCentral.strafeRight(POWER, 8);
        MecanumAutoCentral.backward(POWER, 6);
        MecanumAutoCentral.strafeRight(POWER, 16);
        MecanumAutoCentral.backward(POWER, 12);
        MecanumAutoCentral.strafeLeft(POWER, 75);
    }
}
