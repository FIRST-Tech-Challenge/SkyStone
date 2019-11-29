package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.All.Lift;
import org.firstinspires.ftc.teamcode.TeleOp.ToggleButtons.GamepadButtons;
import org.firstinspires.ftc.teamcode.TeleOp.ToggleButtons.OnOffButton;

import java.util.ArrayList;

/*
 * -Gamepad A:
 *  -Left Joystick controls Left Wheel
 *  -Right Joystick controls Right Wheel
 *  -Left Bumper strafes left
 *  -Right Bumper strafes right
 *  -Y & B controls servo1
 *  -X & A controls servo2
 * -Gamepad B:
 *  -Y and A to intake and outake
 *  -Left joystick and Right joysticks to control the lift
 *  -Adjust the HardwareMap configurations in HardwareMap.java class
 *  -Adjust Intake/Lift power in TeleopConstants.java class
 */

@TeleOp(name = "TeleOp", group = "Linear Opmode")
public class Teleop extends LinearOpMode {
    boolean intake = false;
    boolean outake = false;
    final double normalSpeed = TeleopConstants.drivePowerNormal;
    final double turboSpeed = TeleopConstants.drivePowerTurbo;
    final double slowSpeed = TeleopConstants.drivePowerSlow;
    final double turnSpeed = TeleopConstants.turnPower;
    boolean runLogic = false;

    ArrayList<OnOffButton> buttonLogic = new ArrayList<>();

    boolean blocker = false;
    ArrayList<String> kVData = new ArrayList<>();

    FourWheelMecanumDrivetrain drivetrain;
    Lift lift;

    public void runOpMode() {
        HardwareMap hwMap = new HardwareMap(hardwareMap);

        runLogic = true;

        drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);

        drivetrain.setSpeedMultiplier(normalSpeed);
        drivetrain.resetEncoders();

        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = new Lift(hardwareMap);

        lift.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setSpeedMultiplier(TeleopConstants.liftPower);

        lift.resetEncoders();

        lift.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);

        buttonLogic.add(new OnOffButton(gamepad2, gamepad2, GamepadButtons.A, GamepadButtons.B, //Intake-A & B
                new DcMotor[] { hwMap.leftIntake, hwMap.rightIntake },
                new double[][] { {-TeleopConstants.intakePower, 0}, {TeleopConstants.intakePower, 0} },
                new double[] { TeleopConstants.intakePower, -TeleopConstants.intakePower }));
        buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.Y, new Servo[] { hwMap.foundationLock },   //Foundation Lock-Y
                new double[][] { {TeleopConstants.foundationLockLock, TeleopConstants.foundationLockUnlock} }));
        buttonLogic.add(new OnOffButton(gamepad2, gamepad2, GamepadButtons.LEFT_BUMPER, GamepadButtons.RIGHT_BUMPER,   //Transfer Lock-L & R Bumpers
                new Servo[] { hwMap.transferLock }, new double[][] { {TeleopConstants.transferLockPosUp,
                TeleopConstants.transferLockPosPlatform} },
                new double[] { TeleopConstants.transferLockPosFoundationGrabing }));
        buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.DPAD_DOWN, new Servo[] {hwMap.liftOdometer, hwMap.plateLifter},    //Odometer Lock and Plate Lifter-DPAD DOwn
                new double[][] { {TeleopConstants.odometerLockPosUp, TeleopConstants.odometerLockPosDown},
                        {TeleopConstants.plateLifterPosUp, TeleopConstants.plateLifterPosDown} }));
        buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.LEFT_BUMPER, new Servo[] {hwMap.clawServo1, hwMap.clawServo2}, //Claw Servos-X
                new double[][] { {TeleopConstants.clawServo1PosOpen, TeleopConstants.clawServo1PosClose},
                        {TeleopConstants.clawServo2PosOpen, TeleopConstants.clawServo2PosClose} }));
        buttonLogic.add(new OnOffButton(gamepad2, GamepadButtons.X, new Servo[] { hwMap.transferHorn },   //Foundation Lock-Y
                new double[][] { {TeleopConstants.transferHornPosPush, TeleopConstants.transferHornPosReady} }));

        telemetry.addData("Status", "Ready");

        waitForStart();


        while (opModeIsActive()) {

            //double turn = (-1) * (gamepad1.left_trigger - gamepad1.right_trigger) * turnSpeed;

            //------------------------------===Intake/Outake===------------------------------------------

            //intakeLogic(hwMap);

            for(OnOffButton onOffButton : buttonLogic) {
                onOffButton.getGamepadStateAndRun();

                if(onOffButton.controllingServos() && onOffButton.getServos()[0].getDeviceName()
                        .equalsIgnoreCase("transferHorn") && onOffButton.isRunning()[0]) {
                    sleep(400);
                    hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosTucked);
                } else if(onOffButton.controllingServos() && onOffButton.getServos()[0].getDeviceName()
                        .equalsIgnoreCase("transferHorn") && !onOffButton.isRunning()[0]) {
                    hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosExtended);
                }
            }


            //------------------------------===Linear Sliders===------------------------------------------

            if (gamepad2.right_stick_y != 0) {
                lift.moveLift(gamepad2.right_stick_y);
            } else {
                lift.stop();
            }
            lift.detectResetEncoder();

            //------------------------------===Driving/Strafing===------------------------------------------

            if (!(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0)) {

                double speed;

                if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                    speed = 0;
                } else if (gamepad1.left_stick_y == 0) {
                    speed = Math.abs(gamepad1.left_stick_x);
                } else if (gamepad1.left_stick_x == 0) {
                    speed = Math.abs(gamepad1.left_stick_y);
                } else {
                    speed = (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y)) / 2;
                }

                double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                drivetrain.MoveAngle(speed, angle, 0);
            }

            if(gamepad1.right_stick_x != 0){
                if(gamepad1.right_stick_x > 0)
                    drivetrain.rotate(normalSpeed * gamepad1.right_stick_x, true);
                else if(gamepad1.right_stick_x < 0)
                    drivetrain.rotate(normalSpeed * Math.abs(gamepad1.right_stick_x), false);
            }

            if(gamepad1.left_trigger >= 0.5){
                drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if(gamepad1.right_trigger >= 0.5){
                drivetrain.setPowerAll(turboSpeed);
            }

            if(gamepad1.dpad_up){
                drivetrain.setPowerAll(slowSpeed);
            } else if(gamepad1.dpad_down){
                drivetrain.setPowerAll(-slowSpeed);
            } else if(gamepad1.dpad_right){
                drivetrain.strafe(slowSpeed, true);
            } else if(gamepad1.dpad_left){
                drivetrain.strafe(slowSpeed, false);
            }

            if(gamepad1.left_stick_x == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_y == 0 &&
                    gamepad1.right_stick_y == 0 && gamepad1.right_trigger == 0 &&
                    !gamepad1.dpad_up && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_down){
                drivetrain.stop();
            }

            //------------------------------===Servos===------------------------------------------

            if (gamepad1.y)  //Change servo positions in TeleopConstants.java
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);
            else if (gamepad1.b)
                hwMap.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);

            if (gamepad1.x)
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
            else if (gamepad1.a)
                hwMap.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);

            if (gamepad2.left_bumper) {
                hwMap.liftOdometer.setPosition(TeleopConstants.odometerLockPosUp);
            } else if (gamepad2.right_bumper) {
                hwMap.liftOdometer.setPosition(TeleopConstants.odometerLockPosDown);
            }

            if (gamepad1.left_bumper) {
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);
            } else if (gamepad1.right_bumper) {
                hwMap.transferLock.setPosition(TeleopConstants.transferLockPosPlatform);
            }

            if (gamepad1.left_bumper) {
                StringBuilder sb = new StringBuilder();
                for (String row : kVData) {
                    sb.append(row);
                    sb.append("\n");
                }

                //DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/kV_regression_data_" + System.currentTimeMillis() + ".csv", sb.toString());
                //break;
            }

            telemetry.addData("LeftForwardOdometry", hwMap.leftIntake.getCurrentPosition());
            telemetry.addData("RightForwardOdometry", hwMap.liftTwo.getCurrentPosition());
            telemetry.addData("SidewaysOdometry", hwMap.rightIntake.getCurrentPosition());

            telemetry.addData("FrontLeft", hwMap.frontLeft.getCurrentPosition());
            telemetry.addData("BackLeft", hwMap.backLeft.getCurrentPosition());
            telemetry.addData("FrontRight", hwMap.frontRight.getCurrentPosition());
            telemetry.addData("BackRight", hwMap.backRight.getCurrentPosition());

            telemetry.addData("LiftOne", hwMap.liftOne.getCurrentPosition());
            telemetry.update();
        }


    }

    private void intakeLogic(HardwareMap hwMap){
        if (gamepad2.y && !blocker) {
            if (!intake) {
                intake = true;
                outake = false;
            } else {
                intake = false;
                outake = false;
            }
            blocker = true;
        }

        if (gamepad2.a && !blocker) {
            if (!outake) {
                intake = false;
                outake = true;
            } else {
                intake = false;
                outake = false;
            }
            blocker = true;
        }

        if(!gamepad2.y && !gamepad2.a){
            blocker = false;
        }

        if (intake) {
            hwMap.rightIntake.setPower(-TeleopConstants.intakePower);
            hwMap.leftIntake.setPower(TeleopConstants.intakePower);
        }

        if (outake) {
            hwMap.rightIntake.setPower(TeleopConstants.intakePower);
            hwMap.leftIntake.setPower(-TeleopConstants.intakePower);
        }

        if (!intake && !outake) {
            hwMap.rightIntake.setPower(0);
            hwMap.leftIntake.setPower(0);
        }
    }

    private void saveDataLoop(HardwareMap hw) {
        Thread loop = new Thread() {
            public void run() {
                long initTime = System.currentTimeMillis();
                while (runLogic) {
                    kVData.add(hw.frontLeft.getPower() + "," + hw.backLeft.getPower() + "," +
                            hw.frontLeft.getCurrentPosition() + "," + hw.backLeft.getCurrentPosition() + "," +
                            (System.currentTimeMillis() - initTime));
                    try {
                        sleep(50);
                    } catch (Exception e) {
                    }
                }
            }
        };
        loop.start();
    }
}
