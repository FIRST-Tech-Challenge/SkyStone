//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.subsystems.DriveFunctions;

import org.firstinspires.ftc.teamcode.subsystems.dunk.Dunk;
import org.firstinspires.ftc.teamcode.subsystems.dunk.dunkMinerals;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hang;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.intakeMinerals;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.Flip;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.mineralFlip;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.*;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.*;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

@Autonomous(name="AutoBox") //Name the program
public class autoTensorBox extends LinearOpMode
{
    //Define drive motors
    DcMotor leftMotorFront;
    DcMotor rightMotorFront;
    DcMotor leftMotorBack;
    DcMotor rightMotorBack;

    //Define glyph motors
    DcMotor mineralSpool;
    DcMotor spinner;
    DcMotor lifter;
    DcMotor hanger;

    Servo mineralFlipper;
    Servo dunker;
    Servo markerDropper;

    TeamMarker teamMarker;

    ColorSensor colorSensor;
    RevTouchSensor touch;

    BNO055IMU boschIMU;

    //Define drive powers to avoid magic numbers
    float drivePower = (float) 0.6;
    float shiftPower = (float) 0.6;
    float turnPower = (float) 0.6;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private TensorFlow tensor;

    private TensorFlow.goldMineral goldMineral;
    private ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //Subsystems
    Flip flip;
    Dunk dunk;
    Intake intake;
    Hang hang;
    //***************************************************************************************************************************
    //MAIN BELOW

    @Override
    public void runOpMode() throws InterruptedException {
        //Hardware Map
        leftMotorFront = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        leftMotorBack = hardwareMap.dcMotor.get("leftMotorBack");
        rightMotorBack = hardwareMap.dcMotor.get("rightMotorBack");

        mineralSpool = hardwareMap.dcMotor.get("mineralSpool");
        spinner = hardwareMap.dcMotor.get("spinner");
        lifter = hardwareMap.dcMotor.get("lifter");
        hanger = hardwareMap.dcMotor.get("hanger");

        mineralFlipper = hardwareMap.servo.get("mineralFlipper");
        dunker = hardwareMap.servo.get("dunker");
        markerDropper = hardwareMap.servo.get("markerDropper");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        touch = hardwareMap.get(RevTouchSensor.class, "touch");

        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions chassis = new DriveFunctions(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, boschIMU);

        //Construct Subsystems
        teamMarker = new claiming(markerDropper);
        tensor = new twoSampling(telemetry, hardwareMap, vuforia, tfod);

        //Intialize Subsystems
        flip = new mineralFlip(mineralFlipper);
        dunk = new dunkMinerals(lifter, dunker);
        intake = new intakeMinerals(spinner, mineralSpool);
        hang = new linearActuator(hanger);

        //Initializations
        chassis.initializeRobotBrake();

//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start");
//        telemetry.update();

        //Find Gold Mineral after Initialization but before game starts

        teamMarker.hold();
        while (!isStarted())
        {
            //Set goldMineral to gold position found from getMineralTime()
            goldMineral = tensor.getMineral();
        }

        waitForStart();

        //Code to run once play is pressed
        while(opModeIsActive())
        {
            runTime.reset();
            while (runTime.time() < 5500)
            {
                hanger.setPower(1.0);
                goldMineral = tensor.getMineral();
            }
            hanger.setPower(0.0);

            //Shift off lander
            chassis.rightShiftAutonomous(shiftPower, 200);

            //Move forward
            chassis.driveAutonomous(drivePower, 400);

            //Center robot
            chassis.leftShiftAutonomous(shiftPower, 200);

            //Default to right
            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }

            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                //Turn to right mineral
                chassis.leftTurnIMU(turnPower, 39);

                oneMotorEncoder(mineralSpool, 1.0, 1000);

                flip.down();

                oneMotorEncoder(lifter, 1.0, 500);

                dunk.dunkDown();

                runTime.reset();
                while (!touch.isPressed() && runTime.time() < 750)
                {
                    lifter.setPower(-1.0);
                }

                if (touch.isPressed())
                {
                    lifter.setPower(0.0);
                }

//              Stop the motor
                lifter.setPower(0.0);

                intake.start();
                //Move to mineral and intake
                oneMotorEncoder(mineralSpool, (float) 1.0, 2600);

            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                oneMotorEncoder(mineralSpool, 1.0, 1000);

                flip.down();

                oneMotorEncoder(lifter, 1.0, 500);

                dunk.dunkDown();

                runTime.reset();
                while (!touch.isPressed() && runTime.time() < 750)
                {
                    lifter.setPower(-1.0);
                }

                if (touch.isPressed())
                {
                    lifter.setPower(0.0);
                }

//              Stop the motor
                lifter.setPower(0.0);

                intake.start();
                oneMotorEncoder(mineralSpool, (float) 1.0, 900);
            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {
                //Turn to right mineral
                chassis.rightTurnIMU(turnPower, -45);
                oneMotorEncoder(mineralSpool, 1.0, 1000);

                flip.down();

                oneMotorEncoder(lifter, 1.0, 500);


                dunk.dunkDown();

                runTime.reset();
                while (!touch.isPressed() && runTime.time() < 750)
                {
                    lifter.setPower(-1.0);
                }

                if (touch.isPressed())
                {
                    lifter.setPower(0.0);
                }

//              Stop the motor
                lifter.setPower(0.0);

                intake.start();
                oneMotorEncoder(mineralSpool, (float) 1.0, 2600);
            }

            flip.up();
            while (!chassis.iSeeAColor(colorSensor))
            {
                mineralSpool.setPower(-1.0);
            }
            while (!chassis.isYellow(colorSensor))
            {
                mineralSpool.setPower(-1.0);
            }
            mineralSpool.setPower(0.0);
            flip.flip();
            chassis.leftTurnIMU(turnPower, 90);

            intake.stop();

            chassis.rightShiftAutonomous(shiftPower/2, 350);

            //Back into depot
            chassis.driveAutonomous(drivePower, 1000);

            chassis.leftTurnIMU(turnPower, 135);

            chassis.rightShiftAutonomous(shiftPower / 2, 1000);

            chassis.leftShiftAutonomous(shiftPower / 2, 150);

            //Back into depot
            chassis.driveAutonomous(-drivePower, -1300);

            chassis.leftShiftAutonomous(shiftPower/ 2, 150);

            //Drop marker in depot
            teamMarker.drop();

            chassis.driveAutonomous(-drivePower, -200);


            chassis.rightShiftAutonomous(shiftPower, 100);

            //Drive into crater
            chassis.driveAutonomous(drivePower, 2150);

//            intake.spoolOut(1500);

            oneMotorEncoder(mineralSpool,1.0, 1500);

            flip.down();
            intake.reverse();

            //Always call idle() at the bottom of your while(opModeIsActive()) loop
            idle();

            break;
        }
    }
}