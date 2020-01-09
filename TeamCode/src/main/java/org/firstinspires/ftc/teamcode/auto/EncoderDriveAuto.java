package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Autonomous(name="Encoder Auto")
@Disabled
public class EncoderDriveAuto extends LinearOpMode {


    DcMotorEx tl, tr, bl, br;
    DcMotor front_left, front_right, back_left, back_right, intake_left, intake_right;

    @Override
    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.dcMotor.get("front_left"); // Port 0
        front_right = hardwareMap.dcMotor.get("front_right"); // Port 1
        back_left = hardwareMap.dcMotor.get("back_left"); // Port 2
        back_right = hardwareMap.dcMotor.get("back_right"); // Port 3
        intake_left = hardwareMap.dcMotor.get("intake_left");
        intake_right = hardwareMap.dcMotor.get("intake_right");

        //PID Coefficients for tuning
       /* PIDCoefficients tunedConstants = new PIDCoefficients(0.0025, 0.1, 0.2);
        front_left.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);
        front_right.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);
        back_left.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);
        back_right.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);*/



        front_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorTargets(-112, 112, 112, -112);

        front_left.setPower(1);
        front_right.setPower(1);
        back_left.setPower(1);
        back_right.setPower(1);
        while (front_left.isBusy()) {
            Thread.yield();
        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        front_right.setMode(mode);
        front_left.setMode(mode);
        back_left.setMode(mode);
        back_right.setMode(mode);
    }

    private void setMotorTargets(int tlTarget, int trTarget, int blTarget, int brTarget) {
        front_right.setTargetPosition(trTarget);
        front_left.setTargetPosition(tlTarget);
        back_left.setTargetPosition(blTarget);
        back_right.setTargetPosition(brTarget);
    }

}
