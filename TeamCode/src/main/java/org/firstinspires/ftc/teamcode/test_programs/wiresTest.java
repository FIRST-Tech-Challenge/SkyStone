package org.firstinspires.ftc.teamcode.test_programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

@TeleOp(name="wires test ") //Name the class
public class wiresTest extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor dumper;
    DcMotor intakeLeft;
    DcMotor intakeRight;

    //Define drive powers to avoid magic numbers
    float power = (float) 0.5;
    int degrees = 1000;

//***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        dumper = hardwareMap.dcMotor.get("dumper");
        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");


        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive())
        {
            if (gamepad1.y)
            {
                oneMotorEncoder(leftMotorFront, power, degrees);
                telemetry.addData(" left motor front forwards", leftMotorFront);
            }
            if (gamepad1.b)
            {
                oneMotorEncoder(rightMotorFront, power, degrees);
                telemetry.addData("right motor front forwards", rightMotorFront);
            }
            if (gamepad1.a)
            {
                oneMotorEncoder(leftMotorBack, power, degrees);
                telemetry.addData("left motor back forwards", leftMotorBack);
            }
            if (gamepad1.x)
            {
                oneMotorEncoder(rightMotorBack, power, degrees);
                telemetry.addData("right motor back forwards", rightMotorBack);
            }

            if (gamepad1.dpad_up)
            {
                oneMotorEncoder(leftMotorFront, -power, -degrees);
                telemetry.addData("left motor front backwards", leftMotorFront);
            }
            if (gamepad1.dpad_right)
            {
                oneMotorEncoder(rightMotorFront, -power, -degrees);
                telemetry.addData("right motor front backwards", rightMotorFront);
            }
            if (gamepad1.dpad_down)
            {
                oneMotorEncoder(leftMotorBack, -power, -degrees);
                telemetry.addData("left motor back backwards", leftMotorBack);
            }
            if (gamepad1.dpad_left)
            {
                oneMotorEncoder(rightMotorBack, -power, -degrees);
                telemetry.addData("right motor back backwards", rightMotorBack);
            }

            telemetry.update();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
