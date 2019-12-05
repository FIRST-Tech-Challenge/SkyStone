package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Autonomous(name="Encoder Auto")
public class EncoderDriveAuto extends LinearOpMode {


    DcMotorEx tl, tr, bl, br;

    @Override
    public void runOpMode() throws InterruptedException {
        tl = (DcMotorEx)hardwareMap.dcMotor.get("front_left");
        tr = (DcMotorEx)hardwareMap.dcMotor.get("front_right");
        bl = (DcMotorEx)hardwareMap.dcMotor.get("back_left");
        br = (DcMotorEx)hardwareMap.dcMotor.get("back_right");

        //PID Coefficients for tuning
        PIDCoefficients tunedConstants = new PIDCoefficients(0.0025, 0.1, 0.2);
        tl.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);
        tr.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);
        bl.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);
        br.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, tunedConstants);



        tr.setDirection(DcMotor.Direction.FORWARD);
        tl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorTargets(-112, 112, 112, -112);

        tl.setPower(1);
        tr.setPower(1);
        bl.setPower(1);
        br.setPower(1);
        while (tl.isBusy()) {
            Thread.yield();
        }
        tl.setPower(0);
        tr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        tr.setMode(mode);
        tl.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    private void setMotorTargets(int tlTarget, int trTarget, int blTarget, int brTarget) {
        tr.setTargetPosition(trTarget);
        tl.setTargetPosition(tlTarget);
        bl.setTargetPosition(blTarget);
        br.setTargetPosition(brTarget);
    }

}
