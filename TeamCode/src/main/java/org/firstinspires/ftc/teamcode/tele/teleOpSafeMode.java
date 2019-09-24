package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.DriveFunctions;

import org.firstinspires.ftc.teamcode.subsystems.intake.*;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.*;
import org.firstinspires.ftc.teamcode.subsystems.dunk.*;
import org.firstinspires.ftc.teamcode.subsystems.hang.*;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

@Disabled
@TeleOp(name=" Safe TeleOp") //Name the class
public class teleOpSafeMode extends LinearOpMode {
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    DcMotor mineralSpool;
    DcMotor spinner;
    DcMotor lifter;
    DcMotor hanger;

    //Define  motors
    Servo markerDropper;
    Servo dunker;
    Servo mineralFlipper;


    ColorSensor colorSensor;

    BNO055IMU boschIMU;

    enum flipPositions { DOWN, UP }
    flipPositions currFlipPos;

    //Subsystems
    Flip flip;
    Dunk dunk;
    Intake intake;
    Hang hang;

    int upDownToggle = 0;

    //***************************************************************************************************************************
    //MAIN BELOW
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        mineralSpool = hardwareMap.dcMotor.get("mineralSpool");
        spinner = hardwareMap.dcMotor.get("spinner");
        lifter = hardwareMap.dcMotor.get("lifter");
        hanger = hardwareMap.dcMotor.get("hanger");

        markerDropper = hardwareMap.servo.get("markerDropper");
        dunker = hardwareMap.servo.get("dunker");
        mineralFlipper = hardwareMap.servo.get("mineralFlipper");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions chassis = new DriveFunctions(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, boschIMU);

        //Set the sensor to active mode
        //Set the directions and modes of the motors.
        chassis.initializeRobotFloat();

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Intialize Subsystems
        flip = new mineralFlip(mineralFlipper);
        dunk = new dunkMinerals(lifter, dunker);
        intake = new intakeMinerals(spinner, mineralSpool);
        hang = new linearActuator(hanger);

        currFlipPos = flipPositions.UP;
        dunk.dunkDown();

        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive()) {
            float drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.65);
            float shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.65);
            float leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.5);
            float rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.5);
            float spoolPower = gamepad2.right_stick_y;
            float liftPower = gamepad1.right_stick_y;

            //Drive if joystick pushed more Y than X on gamepad1 (fast)
            if (Math.abs(drivePower) > Math.abs(shiftPower))
            {
                chassis.driveTeleop(drivePower);
            }

            //Shift if pushed more on X than Y on gamepad1 (fast)
            if (Math.abs(shiftPower) > Math.abs(drivePower))
            {
                chassis.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed on gamepad1, turn left at that power (fast)
            if (leftTurnPower > 0)
            {
                chassis.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed on gamepad1, turn right at that power (fast)
            if (rightTurnPower > 0)
                chassis.rightTurnTeleop(rightTurnPower);

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
            {
                chassis.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
            }

            if (gamepad1.a)
            {
                hang.down();
            }

            if (gamepad1.y)
            {
                hang.up();
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper)
            {
                intake.start();
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper)
            {
                intake.reverse();
            }

            if (Math.abs(spoolPower) > 0.1)
            {
                mineralSpool.setPower(-spoolPower);
            }

            if (Math.abs(spoolPower) <= 0.1)
            {
                mineralSpool.setPower(0.0);
            }

            if (gamepad1.b)
            {
                intake.stop();
            }

            if (gamepad2.a)
            {
                chassis.setDriveMotorPowers(0.0f, 0.0f, 0.0f, 0.0f);
                if (currFlipPos == flipPositions.DOWN)
                {
                    intake.reverse();
                    Thread.sleep(500);
                    intake.start();
                    flip.up();
                    intake.stop();
//                    chassis.oneMotorEncoder(mineralSpool, (float) -1.0, -1200);
                    currFlipPos = flipPositions.UP;
                    while (!chassis.iSeeAColor(colorSensor))
                    {
                        drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.65);
                        shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.65);
                        leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.5);
                        rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.5);

                        //Drive if joystick pushed more Y than X on gamepad1 (fast)
                        if (Math.abs(drivePower) > Math.abs(shiftPower))
                        {
                            chassis.driveTeleop(drivePower);
                        }

                        //Shift if pushed more on X than Y on gamepad1 (fast)
                        if (Math.abs(shiftPower) > Math.abs(drivePower))
                        {
                            chassis.shiftTeleop(shiftPower);
                        }

                        //If the left trigger is pushed on gamepad1, turn left at that power (fast)
                        if (leftTurnPower > 0)
                        {
                            chassis.leftTurnTeleop(leftTurnPower);
                        }

                        //If the right trigger is pushed on gamepad1, turn right at that power (fast)
                        if (rightTurnPower > 0)
                            chassis.rightTurnTeleop(rightTurnPower);

                        //If the joysticks are not pushed significantly shut off the wheels
                        if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
                        {
                            chassis.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
                        }
                        mineralSpool.setPower(-1.0);
                        Thread.sleep(6000);
                        break;
                    }
                    while (!chassis.isYellow(colorSensor))
                    {
                        drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.65);
                        shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.65);
                        leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.5);
                        rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.5);

                        //Drive if joystick pushed more Y than X on gamepad1 (fast)
                        if (Math.abs(drivePower) > Math.abs(shiftPower))
                        {
                            chassis.driveTeleop(drivePower);
                        }

                        //Shift if pushed more on X than Y on gamepad1 (fast)
                        if (Math.abs(shiftPower) > Math.abs(drivePower))
                        {
                            chassis.shiftTeleop(shiftPower);
                        }

                        //If the left trigger is pushed on gamepad1, turn left at that power (fast)
                        if (leftTurnPower > 0)
                        {
                            chassis.leftTurnTeleop(leftTurnPower);
                        }

                        //If the right trigger is pushed on gamepad1, turn right at that power (fast)
                        if (rightTurnPower > 0)
                            chassis.rightTurnTeleop(rightTurnPower);

                        //If the joysticks are not pushed significantly shut off the wheels
                        if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15)
                        {
                            chassis.setDriveMotorPowers((float) 0.0, (float) 0.0, (float) 0.0, (float) 0.0);
                        }
                        mineralSpool.setPower(-1.0);
                        Thread.sleep(6000);
                        break;
                    }
                    mineralSpool.setPower(0.0);
                    flip.flip();
                }
                else if (currFlipPos == flipPositions.UP)
                {
                    oneMotorEncoder(mineralSpool, (float) 1.0, 1000);
                    flip.down();
                    intake.start();
                    currFlipPos=flipPositions.DOWN;
                }
            }

            if (gamepad2.y)
            {
                flip.up();
            }
            if (gamepad2.x)
            {
                flip.down();
            }
            if (gamepad2.b)
            {
                flip.flip();
            }
            //Dunk
            if (gamepad1.dpad_up || gamepad2.dpad_up)
            {
                dunk.dunk();
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down)
            {
                dunk.dunkDown();
            }

            if (Math.abs(liftPower) > 0.1)
            {
                dunk.dunkHold();
                lifter.setPower(liftPower);
            }
            if (Math.abs(liftPower) <= 0.1)
            {
                lifter.setPower(0.0);
            }

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();
        }//Close while opModeIsActive loop
    } //Close "run Opmode" loop
} //Close class and end program
