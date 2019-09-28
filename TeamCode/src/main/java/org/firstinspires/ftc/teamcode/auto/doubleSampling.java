//Run from the package
package org.firstinspires.ftc.teamcode.auto;

//Import necessary items

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.subsystems.team_marker.TeamMarker;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.claiming;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.TensorFlow;
import org.firstinspires.ftc.teamcode.subsystems.tensorFlow.twoSampling;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;
@Disabled
@Autonomous(name="DoubleSampling") //Name the program
public class doubleSampling extends LinearOpMode
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
    TensorFlow tensor;
    private ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    TensorFlow.goldMineral goldMineral;

    //Subsystems
    Flip flip;
    Dunk dunk;
    Intake intake;
    Hang hang;
//***************************************************************************************************************************
    //MAIN BELOW

    @Override
    public void runOpMode() throws InterruptedException
    {
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

        mineralSpool.setDirection(DcMotorSimple.Direction.REVERSE);

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

            oneMotorEncoder(mineralSpool, 1.0, 1000);

            flip.down();

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


            //Default to right
            if (goldMineral == TensorFlow.goldMineral.UNKNOWN)
            {
                goldMineral = TensorFlow.goldMineral.RIGHT;
            }

            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                //Turn to right mineral
                chassis.leftTurnIMU(turnPower, 42);
                //Move to mineral and intake
                oneMotorEncoder(mineralSpool, (float) 1.0, 2600);

            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                oneMotorEncoder(mineralSpool, (float) 1.0, 900);
            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {
                //Turn to right mineral
                chassis.rightTurnIMU(turnPower, -45);
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
            chassis.rightTurnIMU(turnPower, -90);

            intake.stop();

            chassis.leftShiftAutonomous(shiftPower / 2, 350);

            //Back up towards wall
            chassis.driveAutonomous(-drivePower, -1900);

            //Become parallel with wall facing crater
            chassis.rightTurnIMU(turnPower, -45);

            chassis.leftShiftAutonomous(shiftPower / 2, 500);

            chassis.rightShiftAutonomous(shiftPower / 2, 150);

            //Back into depot
            chassis.driveAutonomous(-drivePower, -1500);

            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {
                teamMarker.drop();
            }

            chassis.driveAutonomous(-drivePower/ 2, -200);

            chassis.rightShiftAutonomous(shiftPower/2, 600);

            if (goldMineral == TensorFlow.goldMineral.LEFT)
            {
                chassis.rightTurnIMU(turnPower, -135);
                teamMarker.drop();

            }
            if (goldMineral == TensorFlow.goldMineral.CENTER)
            {
                chassis.rightTurnIMU(turnPower, -90);
                teamMarker.drop();
            }
            if (goldMineral == TensorFlow.goldMineral.RIGHT)
            {

            }
            flip.down();

            intake.start();

            oneMotorEncoder(mineralSpool, 1.0, 2500);

            intake.stop();

            //Spool into crater

            idle();
            break;
        }
    }
}