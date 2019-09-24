//Run from the package
package org.firstinspires.ftc.teamcode.test_programs;

//Import necessary items

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.DriveFunctions;
@Disabled
@TeleOp(name = "Data Logging Program") //Name the program
public class dataLogging extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor mineralSpool;
    DcMotor lifter;
    DcMotor spinner;
    DcMotor hanger;

    Servo mineralFlipper;
    Servo dunker;
    Servo markerDropper;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        mineralSpool = hardwareMap.dcMotor.get("mineralSpool");
        lifter = hardwareMap.dcMotor.get("lifter");
        spinner = hardwareMap.dcMotor.get("spinner");
        hanger = hardwareMap.dcMotor.get("hanger");

        mineralFlipper = hardwareMap.servo.get("mineralFlipper");
        dunker = hardwareMap.servo.get("dunker");
        markerDropper = hardwareMap.servo.get("markerDropper");

        waitForStart();

//***************************************************************************************************************************
        //While the op mode is active, loop and read the RGB data.
        //Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())
        {
            //If b is pressed, reset the encoders
            if (gamepad1.b)
            {
                //Reset the encoders
                leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //Use the encoders
                leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            //Show all the encoder values on the driver station
            telemetry.addData("left front", leftMotorFront.getCurrentPosition());
            telemetry.addData("left back", leftMotorBack.getCurrentPosition());
            telemetry.addData("right front", rightMotorFront.getCurrentPosition());
            telemetry.addData("right back", rightMotorBack.getCurrentPosition());
            telemetry.addData("hanger", hanger.getCurrentPosition());
            telemetry.addData("lifter", lifter.getCurrentPosition());

            //Update the data if/when it changes
            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

        } //Close "while(opModeIsActive())" loop
    } //Close "run Opmode" loop
} //Close class and end program