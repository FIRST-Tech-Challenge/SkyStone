package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveFunctions;

import org.firstinspires.ftc.teamcode.subsystems.intake.*;
import org.firstinspires.ftc.teamcode.subsystems.mineral_flip.*;
import org.firstinspires.ftc.teamcode.subsystems.dunk.*;
import org.firstinspires.ftc.teamcode.subsystems.hang.*;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.TeamMarker;
import org.firstinspires.ftc.teamcode.subsystems.team_marker.claiming;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

@Disabled
@TeleOp(name="TeleOp") //Name the class
public class teleMaster extends LinearOpMode {
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
    RevTouchSensor touch;

    BNO055IMU boschIMU;

    enum flipPositions {DOWN, UP}

    flipPositions currFlipPos;

    //Subsystems
    Flip flip;
    Dunk dunk;
    Intake intake;
    Hang hang;
    TeamMarker teamMarker;

    double MAX_POWER = 1.0;
    double STOP_POWER = 0.0;
    private ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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
        touch = hardwareMap.get(RevTouchSensor.class, "touch");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        boschIMU = hardwareMap.get(BNO055IMU.class, "boschIMU");

        //Set up the DriveFunctions class and give it all the necessary components (motors, sensors)
        DriveFunctions chassis = new DriveFunctions(DcMotor.ZeroPowerBehavior.BRAKE, leftMotorFront, rightMotorFront, leftMotorBack, rightMotorBack, boschIMU);


        //Intialize Subsystems
        flip = new mineralFlip(mineralFlipper);
        dunk = new dunkMinerals(lifter, dunker);
        intake = new intakeMinerals(spinner, mineralSpool);
        hang = new linearActuator(hanger);
        teamMarker = new claiming(markerDropper);

        currFlipPos = flipPositions.UP;
        dunk.dunkDown();
        teamMarker.hold();


        //Wait for start button to be clicked
        waitForStart();

//***************************************************************************************************************************
        while (opModeIsActive()) {
            float drivePower = (float) ((gamepad1.left_stick_y + gamepad2.left_stick_y) * 0.6);
            float shiftPower = (float) ((gamepad1.left_stick_x + gamepad2.left_stick_x) * 0.6);
            float leftTurnPower = (float) ((gamepad1.left_trigger + gamepad2.left_trigger) * 0.6);
            float rightTurnPower = (float) ((gamepad1.right_trigger + gamepad2.right_trigger) * 0.6);
            float spoolPower = gamepad1.right_stick_y;
            float liftPower = - gamepad2.right_stick_y;

            //Drive if joystick pushed more Y than X on gamepad1 (fast)
            if (Math.abs(drivePower) > Math.abs(shiftPower)) {
                chassis.driveTeleop(drivePower);
            }

            //Shift if pushed more on X than Y on gamepad1 (fast)
            if (Math.abs(shiftPower) > Math.abs(drivePower)) {
                chassis.shiftTeleop(shiftPower);
            }

            //If the left trigger is pushed on gamepad1, turn left at that power (fast)
            if (leftTurnPower > 0) {
                chassis.leftTurnTeleop(leftTurnPower);
            }

            //If the right trigger is pushed on gamepad1, turn right at that power (fast)
            if (rightTurnPower > 0)
                chassis.rightTurnTeleop(rightTurnPower);

            //If the joysticks are not pushed significantly shut off the wheels
            if (Math.abs(drivePower) + Math.abs(shiftPower) + Math.abs(leftTurnPower) + Math.abs(rightTurnPower) < 0.15) {
                chassis.stopDriving();
            }

            if (gamepad1.a)
            {
                flip.up();
                hang.down();
            }

            if (gamepad1.y) {
                hang.up();
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intake.start();
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intake.reverse();
            }

            if (Math.abs(spoolPower) > 0.1) {
                mineralSpool.setPower(- spoolPower);
            }

            if (Math.abs(spoolPower) <= 0.1)
            {
                mineralSpool.setPower(STOP_POWER);
            }

            if (gamepad1.b)
            {
                intake.stop();
            }

            if (gamepad2.a)
            {
                chassis.stopDriving();
                if (currFlipPos == flipPositions.DOWN)
                {
                    intake.reverse();
                    Thread.sleep(300);
                    intake.start();
                    flip.up();
                    intake.reverse();
                    currFlipPos = flipPositions.UP;
                    dunk.dunkDown();
                    chassis.spoolInFully(mineralSpool, colorSensor, gamepad1, gamepad2);
                    mineralSpool.setPower(STOP_POWER);
                    flip.flip();
                    Thread.sleep(1200);
                    flip.down();
                    dunk.dunkHold();
                    chassis.omeWithDriveMotors(lifter, MAX_POWER, 4400, gamepad1, gamepad2);
                    lifter.setPower(0.1);
                }
                else if (currFlipPos == flipPositions.UP)
                {
                    flip.down();
                    intake.reverse();
                    chassis.omeWithDriveMotors(mineralSpool, MAX_POWER, 500, gamepad1, gamepad2);
                    intake.start();
                    currFlipPos = flipPositions.DOWN;
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
            if (gamepad2.dpad_up)
            {
                dunk.dunk();
                chassis.omeWithDriveMotors(mineralSpool, MAX_POWER, 200, gamepad1, gamepad2);
                lifter.setPower(STOP_POWER);

                Thread.sleep(750);

                dunk.dunkDown();

                lifter.setPower(-MAX_POWER);

                runTime.reset();
                while (! touch.isPressed() && runTime.time() < 3000)
                {
                    chassis.chassisTeleOp(gamepad1, gamepad2);
                    lifter.setPower(-MAX_POWER);
                    mineralSpool.setPower(MAX_POWER / 4);
                }
                chassis.stopDriving();

                if (touch.isPressed())
                {
                    lifter.setPower(STOP_POWER);
                }

                mineralSpool.setPower(STOP_POWER);

//              Stop the motor
                lifter.setPower(STOP_POWER);
            }

            if (gamepad2.dpad_down)
            {
                dunk.dunkDown();
            }

            if (liftPower > 0.1)
            {
                dunk.dunkHold();
                lifter.setPower(liftPower);
            }
            if (liftPower < -0.1)
            {
                dunk.dunkDown();
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
}
