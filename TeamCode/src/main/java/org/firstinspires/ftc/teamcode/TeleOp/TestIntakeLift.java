package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.All.HardwareMap;

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
 *  -Adjust Intake/Lift power in teleopConstants.java class
 */

@TeleOp(name="Test Intake & Lift", group="Linear Opmode")
public class TestIntakeLift extends LinearOpMode {
    boolean intake = false;
    boolean outake = false;
    final double slowSpeed = 0.7;
    final double turnSpeed = 0.6;
    boolean turningTowards = false;
    boolean runLogic = false;

    ArrayList<String> kVData = new ArrayList<>();

    FourWheelMecanumDrivetrain drivetrain;
    public void runOpMode(){
        HardwareMap hwMap = new HardwareMap(hardwareMap);

        runLogic = true;
        saveDataLoop(hwMap);

        drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        drivetrain.setSpeedMultiplier(slowSpeed);
        drivetrain.resetEncoders();

        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hwMap.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hwMap.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Ready");

        waitForStart();

        while (opModeIsActive()){

            double turn = (-1) * (gamepad1.left_trigger - gamepad1.right_trigger) * turnSpeed;

            if (isStopRequested()) {
                StringBuilder sb = new StringBuilder();
                for (String row : kVData) {
                    sb.append(row);
                    sb.append("\n");
                }
            }

            //------------------------------===Intake/Outake===------------------------------------------

            if(gamepad2.y) {
                if(!intake) {
                    intake = true;
                    outake = false;
                } else {
                    intake = false;
                    outake = false;
                }
            }

            if(gamepad2.a) {
                if(!outake) {
                    intake = false;
                    outake = true;
                } else {
                    intake = false;
                    outake = false;
                }
            }

            if(intake){
                hwMap.rightIntake.setPower(-teleopConstants.intakePower);
                hwMap.leftIntake.setPower(teleopConstants.intakePower);
            }

            if(outake){
                hwMap.rightIntake.setPower(teleopConstants.intakePower);
                hwMap.leftIntake.setPower(-teleopConstants.intakePower);
            }

            if(!intake && !outake){
                hwMap.rightIntake.setPower(0);
                hwMap.leftIntake.setPower(0);
            }

            //------------------------------===Linear Sliders===------------------------------------------

            if(gamepad2.left_stick_y == 1){
                hwMap.liftOne.setPower(teleopConstants.liftPower);
            } else if(gamepad2.left_stick_y == -1){
                hwMap.liftOne.setPower(-teleopConstants.liftPower);
            } else {
                hwMap.liftOne.setPower(0);
            }

            if(gamepad2.right_stick_y == 1){
                hwMap.liftTwo.setPower(-teleopConstants.liftPower);
            } else if(gamepad2.right_stick_y == -1){
                hwMap.liftTwo.setPower(teleopConstants.liftPower);
            } else {
                hwMap.liftTwo.setPower(0);
            }

            //------------------------------===Driving/Strafing===------------------------------------------



            if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0)) {



                double speed;

                if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                    speed = 0;
                }
                else if ( gamepad1.right_stick_y == 0 ) {
                    speed = Math.abs(gamepad1.left_stick_x) ;
                }
                else if ( gamepad1.left_stick_x == 0 ) {
                    speed = Math.abs(gamepad1.right_stick_y) ;
                }
                else {
                    speed = ( Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_y) ) / 2;
                }

                double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.right_stick_y);
                drivetrain.MoveAngle(speed, angle, turn);
            } else {
                drivetrain.stop();
            }

            //------------------------------===Servos===------------------------------------------

            if(gamepad1.y)  //Change servo positions in teleopConstants.java
                hwMap.clawServo1.setPosition(teleopConstants.clawServo1Pos1);
            else if(gamepad1.b)
                hwMap.clawServo1.setPosition(teleopConstants.clawServo1Pos2);

            if(gamepad1.x)
                hwMap.clawServo2.setPosition(teleopConstants.clawServo2Pos1);
            else if(gamepad1.a)
                hwMap.clawServo2.setPosition(teleopConstants.clawServo2Pos2);

            if(gamepad2.left_bumper) {
                hwMap.liftOdometer.setPosition(teleopConstants.odometerLockPos);
            } else if(gamepad2.right_bumper) {
                hwMap.liftOdometer.setPosition(-teleopConstants.odometerLockPos);
            }

            if(gamepad1.left_bumper) {
                hwMap.transferLock.setPosition(teleopConstants.transferLockPos);
            } else if(gamepad1.right_bumper) {
                hwMap.transferLock.setPosition(-teleopConstants.transferLockPos);
            }

            if(gamepad1.left_bumper) {
                String temp = kVData.toString();
                temp = temp.replaceAll("]","");
                temp = temp.replaceAll("\\[","");
                DriveConstant.writeFile(AppUtil.ROOT_FOLDER + "/RoadRunner/kV_regression_data.csv", temp);
                runLogic = false;
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

    public void saveDataLoop(HardwareMap hw){
        Thread loop = new Thread() {
            public void run() {
                while(runLogic) {
                    kVData.add(hw.frontLeft.getPower() + "," + hw.backLeft.getPower() + "," +
                            "," + hw.frontLeft.getCurrentPosition() + "," + hw.backLeft.getCurrentPosition());
                    try {
                        sleep(50);
                    } catch (Exception e){}
                }
            }
        };
        loop.start();
    }
}
