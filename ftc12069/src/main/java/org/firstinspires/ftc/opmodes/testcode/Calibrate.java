package org.firstinspires.ftc.opmodes.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Calibrate", group = "auto")
public class Calibrate extends LinearOpMode {
    //
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    //Calculate encoder conversion
    private Integer cpr = 28; //counts per rotation
    private Integer gearratio = 2 / 1;
    private Double diameter = 4.125;
    private Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference
    private Double bias;//adjust until your robot goes 20 inches
    //
    private Double conversion;

    {
        conversion = cpi * bias;
    }

    public Calibrate() {
        bias = 0.8;
    }

    //
    public void runOpMode() {
        //
        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        backright.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        //
        waitForStartify();
        //
        moveToPosition();//Don't change this line, unless you want to calibrate with different speeds
        //
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    private void moveToPosition() {
        //
        int move1 = (int) (Math.round(((double) 20 - 5) * conversion));
        int movefl2 = frontleft.getCurrentPosition() + (int) (Math.round((double) 20 * conversion));
        int movefr2 = frontright.getCurrentPosition() + (int) (Math.round((double) 20 * conversion));
        int movebl2 = backleft.getCurrentPosition() + (int) (Math.round((double) 20 * conversion));
        int movebr2 = backright.getCurrentPosition() + (int) (Math.round((double) 20 * conversion));
        //
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move1);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move1);
        backleft.setTargetPosition(backleft.getCurrentPosition() + move1);
        backright.setTargetPosition(backright.getCurrentPosition() + move1);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(0.2);
        frontright.setPower(0.2);
        backleft.setPower(0.2);
        backright.setPower(0.2);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
        }
        //
        frontleft.setTargetPosition(movefl2);
        frontright.setTargetPosition(movefr2);
        backleft.setTargetPosition(movebl2);
        backright.setTargetPosition(movebr2);
        //
        frontleft.setPower(.1);
        frontright.setPower(.1);
        backleft.setPower(.1);
        backright.setPower(.1);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
        }
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

    private void waitForStartify() {
        waitForStart();
    }
}