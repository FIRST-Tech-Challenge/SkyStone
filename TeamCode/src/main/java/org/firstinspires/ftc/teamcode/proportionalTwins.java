package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class proportionalTwins extends LinearOpMode {

    public DcMotor armRaiser;
    public DcMotor armExtender;

    int fullRaise; //number of encoder ticks at max height
    int fullExtend; //number of encoder ticks at max extension

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();

        armRaiser = hardwareMap.get(DcMotor.class, "armRaiser");
        armExtender = hardwareMap.get(DcMotor.class, "armExtender");


        waitForStart();
        while(opModeIsActive()) {


        }
    }
    public boolean toleranceStoppage(DcMotor wheel, int targetPos, double tolerance) {
        int intTolerance = (int) Math.round(tolerance);
        return wheel.getCurrentPosition() <= targetPos-intTolerance;
    }

    public double powerControl(DcMotor motor, int maxEnc) {
        double percentError = (motor.getCurrentPosition()- maxEnc)/maxEnc;
        double percentErrorClip = Range.clip(percentError, 0.0, 1.0);
        return percentErrorClip;
    }

    public int encoderPosNew(DcMotor motor, int maxVal) {
        int encoderPercent = (int) (maxVal*powerControl(motor, maxVal));
        return encoderPercent;
    }

    public void geminiControl(DcMotor raiser, DcMotor extender) { //Method with built in controller
        raiser.setPower(gamepad1.left_trigger);
        int targetPosExtender = encoderPosNew(raiser, fullRaise);

    }
}
