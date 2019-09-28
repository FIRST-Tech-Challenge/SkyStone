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

            opMode.telemetry.addData("Success", "Intake Initialized");
            opMode.telemetry.update();

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
    public void compliantIntake_Auto(double runTime, boolean block)
    {
        time.reset();

        opMode.telemetry.addData("Active", "Intake Running");
        opMode.telemetry.update();

       // driveTrain.encoderDrive(.5, 10, 10, 10, 10, 2);
        //move to block?
        rightSide.setPower(PICKUP);
        leftSide.setPower(PICKUP);

        while(time.seconds() < runTime && block){ //runtime is time it takes for intake to run and pass through

        }

        //move toward block
      //  driveTrain.encoderDrive(.5, -10, -10, -10, -10, 2);

        rightSide.setPower(IDLE);
        leftSide.setPower(IDLE);

        opMode.telemetry.addData("Inactive", "Intake Off");
        opMode.telemetry.update();

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
