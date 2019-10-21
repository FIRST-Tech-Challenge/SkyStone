package org.firstinspires.ftc.teamcode.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    /*
    Compliant Wheel Intake
    Includes Pass Through System
    -------------------------------------------
    Detects a block using vision - in OpClass
    Activates Intake Motors after block found - to save power
    */

    //intake planetary motors
    DcMotor rightSide;
    DcMotor leftSide;

    ElapsedTime time = new ElapsedTime();

    private LinearOpMode opMode;

    private static final double PICKUP = .6;
    private static final double IDLE = 0;

    public boolean initIntake(OpMode opMode)
    {
        this.opMode = (LinearOpMode) opMode;
        time.reset();

        try
        {
            rightSide = opMode.hardwareMap.dcMotor.get("RIn");
            leftSide = opMode.hardwareMap.dcMotor.get("LIn");

            rightSide.setDirection(DcMotor.Direction.REVERSE);
            leftSide.setDirection(DcMotor.Direction.FORWARD);

        } catch (Exception e)
        {
            opMode.telemetry.addData("Failed", "Failed to Map Motors");
            opMode.telemetry.update();

            return false;
        }

        rightSide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftSide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        return true;
    }

    //is to be called in OpMode
    public void autoIntake(DriveTrain drive, double runTime, boolean block)
    {
        time.reset();

        rightSide.setPower(PICKUP);
        leftSide.setPower(PICKUP);

        while(time.seconds() < runTime && block){

        }

        rightSide.setPower(IDLE);
        leftSide.setPower(IDLE);

    }

    public void compliantIntake_TeleOp()
    {
        if(opMode.gamepad2.x) //set game pad button to x, could change, survey people
        {
            rightSide.setPower(PICKUP);
            leftSide.setPower(PICKUP);

            opMode.telemetry.addData("Active", "Intake Running");
            opMode.telemetry.update();
        }else
        {
            rightSide.setPower(IDLE);
            leftSide.setPower(IDLE);
        }

    }
}
