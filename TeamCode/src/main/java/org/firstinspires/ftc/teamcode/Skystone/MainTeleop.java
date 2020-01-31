package org.firstinspires.ftc.teamcode.Skystone;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.MotionAction;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "MainTeleOpSky2", group = "Linear Opmode")
public class MainTeleop extends LinearOpMode {
    Robot robot;

    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    long currentTime;

    boolean onSlowDrive, changedSlowDrive = false;

    boolean isRetract = false;
    boolean isExtend = false;
    boolean isClamp = false;
    boolean is90 = false;
    boolean foundationToggle = false;
    boolean resetfoundation = false;

    public static double powerScaleFactor = 0.9;

    boolean isIntakeMode = false;


    @Override
    public void runOpMode() {
        resetRobot();
        robot.initServos();
        waitForStart();

        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();

        while (opModeIsActive()) {
            telemetry.update();

            robotModeLogic();

            slowDriveLogic();
            driveLogic();

            foundationLogic();

            outtakeLogic();
            capstoneLogic();
            spoolLogic();
            intakeLogic();

            if(gamepad1.a){

                robot.setDrivetrainMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.setDrivetrainMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                position2D.o.resetOdometry();

                robot.moveToPoint(24.5,0,0.5,1,0);
            }

            if (robot.isDebug()) {
                telemetry.addLine("xPos: " + robot.getRobotPos().x);
                telemetry.addLine("yPos: " + robot.getRobotPos().y);
                telemetry.addLine("angle: " + Math.toDegrees(MathFunctions.angleWrap(robot.getAnglePos())));
                telemetry.addLine("XPODLeft " + robot.getfLeft().getCurrentPosition());
                telemetry.addLine("XPODRight " + robot.getfRight().getCurrentPosition());
                telemetry.addLine("YPOD " + robot.getbLeft().getCurrentPosition());
            }

            if (isIntakeMode) {
                telemetry.addLine("CURRENT ROBOT MODE: INTAKE BOT");
            } else {
                telemetry.addLine("CURRENT ROBOT MODE: NORMAL");
            }
        }
    }

    private double lastDropPosition = 0;
    private double spoolTargetPosition = 0;
    private boolean isMovingSpoolToPosition;

    private boolean isG2DPDPushed = false;

    private void spoolLogic() {
        double spoolPower = -gamepad2.left_stick_y;

        if (Math.abs(gamepad2.left_stick_y) >= .1) {
            isMovingSpoolToPosition = false;
        }

        if (gamepad2.left_trigger != 0) {
            spoolPower = .15;
        }

        double spoolPosition = robot.getOuttakeSpool().getCurrentPosition();

        if (!gamepad2.dpad_down && isG2DPDPushed) {
            isMovingSpoolToPosition = true;
            spoolTargetPosition = robot.getOuttakeSpool().getCurrentPosition() - 150;

            isG2DPDPushed = false;
        } else if (gamepad2.dpad_down) {
            isG2DPDPushed = true;
        }

        if (gamepad2.left_bumper) {
            double lastDropLevel = Math.floor((lastDropPosition - robot.SPOOL_FIRSTLEVEL_POSITION) / robot.SPOOL_LEVEL_INCREMENT);

            spoolTargetPosition = robot.SPOOL_FIRSTLEVEL_POSITION + ((lastDropLevel + 1) * robot.SPOOL_LEVEL_INCREMENT) + 150;

            isMovingSpoolToPosition = true;
        }

        if (isMovingSpoolToPosition) {
            if (spoolTargetPosition < 0) {
                spoolTargetPosition = 0;
            }
            if (Math.abs(spoolPosition - spoolTargetPosition) < 25) {
                spoolPower = .15;
            } else if (spoolPosition < spoolTargetPosition) {
                spoolPower = 1;
            } else {
                spoolPower = -1;
            }
        }

        if(robot.getOuttakeSpool().getCurrentPosition() >= 2825){
            if(spoolPower == 0){
                spoolPower += 0.15;
            }
            spoolPower = Math.min(spoolPower/2,0.15);
        }
        if(spoolPower < 0 && robot.getOuttakeSpool().getCurrentPosition() <= 0){
            spoolPower = 0;
        }
        robot.getOuttakeSpool().setPower(spoolPower);
        robot.getOuttakeSpool2().setPower(spoolPower);

        telemetry.addLine("spoolPower: " + spoolPower);
        telemetry.addLine("isMovingSpoolToPosition: " + isMovingSpoolToPosition);
        telemetry.addLine("targetSpoolPosition: " + spoolTargetPosition);
        telemetry.addLine("Spool Position " + spoolPosition);
    }

    private void resetRobot() {
        robot = new Robot(hardwareMap, telemetry, this);

        robot.setDrivetrainMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDrivetrainMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getFrontClamp().setDirection(Servo.Direction.FORWARD);

        robot.getOuttakeSpool().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getOuttakeSpool2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.getOuttakeSpool().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getOuttakeSpool().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getOuttakeSpool2().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.getIntakeLeft().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.getIntakeRight().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //teleop methods
    private void driveLogic() {
        // TODO: change all of this stuff to x, y, and turn movements
        //tank drive
        fLPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * powerScaleFactor;
        fRPower = (-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * powerScaleFactor;
        bLPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * powerScaleFactor;
        bRPower = (-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * powerScaleFactor;
        if (gamepad1.right_trigger != 0) {
            fLPower = (gamepad1.right_trigger) * powerScaleFactor;
            fRPower = (-gamepad1.right_trigger) * powerScaleFactor;
            bLPower = (-gamepad1.right_trigger) * powerScaleFactor;
            bRPower = (gamepad1.right_trigger) * powerScaleFactor;
        } else if (gamepad1.left_trigger != 0) {
            fLPower = (-gamepad1.left_trigger) * powerScaleFactor;
            fRPower = (gamepad1.left_trigger) * powerScaleFactor;
            bLPower = (gamepad1.left_trigger) * powerScaleFactor;
            bRPower = (-gamepad1.left_trigger) * powerScaleFactor;
        }
        //Straight D-Pad move
        if (gamepad1.dpad_up) {
            fLPower = (gamepad1.left_stick_y) + powerScaleFactor;
            bLPower = (gamepad1.left_stick_y) + powerScaleFactor;
            fRPower = (gamepad1.right_stick_y + powerScaleFactor);
            bRPower = (gamepad1.right_stick_y + powerScaleFactor);
        } else if (gamepad1.dpad_down) {
            fLPower = (gamepad1.left_stick_y) - powerScaleFactor;
            bLPower = (gamepad1.left_stick_y) - powerScaleFactor;
            fRPower = (gamepad1.right_stick_y - powerScaleFactor);
            bRPower = (gamepad1.right_stick_y) - powerScaleFactor;
        } else if (gamepad1.dpad_right) {
            fLPower = (gamepad1.right_stick_y) + powerScaleFactor;
            bLPower = (gamepad1.right_stick_y) + powerScaleFactor;
            fRPower = (gamepad1.left_stick_y) - powerScaleFactor;
            bRPower = (gamepad1.left_stick_y) - powerScaleFactor;
        } else if (gamepad1.dpad_left) {
            fRPower = (gamepad1.right_stick_y) + powerScaleFactor;
            bRPower = (gamepad1.right_stick_y) + powerScaleFactor;
            fLPower = (gamepad1.left_stick_y) - powerScaleFactor;
            bLPower = (gamepad1.left_stick_y) - powerScaleFactor;
        }

        robot.allWheelDrive(fLPower, fRPower, bLPower, bRPower);
    }

    private void capstoneLogic() {
        if (gamepad2.right_trigger != 0) {
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED);
            long startTime = SystemClock.elapsedRealtime();

            while(SystemClock.elapsedRealtime() - startTime <= 350){
                robotModeLogic();

                slowDriveLogic();
                driveLogic();

                foundationLogic();

                spoolLogic();
                intakeLogic();
            }


            startTime = SystemClock.elapsedRealtime();
            robot.getFrontClamp().setPosition(robot.FRONTCLAMP_ACTIVATECAPSTONE);

            while(SystemClock.elapsedRealtime() - startTime <= 350){
                robotModeLogic();

                slowDriveLogic();
                driveLogic();

                foundationLogic();

                spoolLogic();
                intakeLogic();
            }
            startTime = SystemClock.elapsedRealtime();
            robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);

            while(SystemClock.elapsedRealtime() - startTime <= 200){
                robotModeLogic();

                slowDriveLogic();
                driveLogic();

                foundationLogic();

                spoolLogic();
                intakeLogic();
            }

            robot.getFrontClamp().setPosition(robot.FRONTCLAMP_CLAMPED);

        }
    }

    private void slowDriveLogic() {
        //toggle driving speed
        if (gamepad1.left_bumper && !changedSlowDrive) {
            powerScaleFactor = (onSlowDrive) ? 0.9 : 0.4;
            onSlowDrive = !onSlowDrive;
            changedSlowDrive = true;
        } else if (!gamepad1.left_bumper) {
            changedSlowDrive = false;
        }

//        if (powerScaleFactor == 0.4) {
////            telemetry.addData("Driving Mode","Slow");
//        } else {
////            telemetry.addData("Driving Mode","Normal");
//        }
    }

    private void intakeLogic() {
        if (Math.abs(gamepad2.left_stick_y) <= 0.25) {
            double intakeLeftPower = 0;
            double intakeRightPower = 0;

            intakeLeftPower = gamepad2.right_stick_y;
            intakeRightPower = gamepad2.right_stick_y;

            robot.getIntakeLeft().setPower(intakeLeftPower);
            robot.getIntakeRight().setPower(intakeRightPower);
            if (Math.abs(gamepad2.right_stick_y) >= 0.25 && !isIntakeMode && !isClamp && !isRetract && !isExtend && !is90 && robot.getOuttakeExtender().getPosition() != robot.OUTTAKE_SLIDE_EXTENDED && robot.getOuttakeExtender().getPosition() != robot.OUTTAKE_SLIDE_PARTIAL_EXTEND) {
                robot.getBackClamp().setPosition(robot.BACKCLAMP_CLAMPED);
                robot.getFrontClamp().setPosition(robot.FRONTCLAMP_RELEASED);
            }
        }

        if (isIntakeMode) {
            robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED - 0.15);
        }
    }

    Queue<MotionAction> outtakeActions = new LinkedList<>();

    private boolean isTogglingG2A = false;
    private boolean isTogglingG2B = false;
    private boolean isTogglingG2X = false;

    private void outtakeLogic() {
        currentTime = SystemClock.elapsedRealtime();
        // Logic to control outtake; with a delay on the pivot so that the slides can extend before pivot rotation
        if (gamepad2.a && !isTogglingG2A) { // Clamp and Extend
            isTogglingG2A = true;

            outtakeActions.clear();

            outtakeActions.add(new MotionAction(robot.getOuttakeExtender(), robot.OUTTAKE_SLIDE_EXTENDED, currentTime + robot.DELAY_SLIDE_ON_EXTEND, robot));
        } else if (!gamepad2.a) {
            isTogglingG2A = false;
        }
        if (gamepad2.b && !isTogglingG2B) { // Deposit and Reset
            lastDropPosition = robot.getOuttakeSpool().getCurrentPosition() + 250;
            isMovingSpoolToPosition = false;

            isTogglingG2B = true;

            outtakeActions.clear();

            outtakeActions.add(new MotionAction(robot.getIntakePusher(), robot.PUSHER_RETRACTED, currentTime + robot.DELAY_PUSHER_ON_RETRACT, robot));
            outtakeActions.add(new MotionAction(robot.getBackClamp(), robot.BACKCLAMP_RELEASED, currentTime + robot.DELAY_RELEASE_CLAMP_ON_RETRACT, robot));
            outtakeActions.add(new MotionAction(robot.getFrontClamp(), robot.FRONTCLAMP_RELEASED, currentTime + robot.DELAY_RELEASE_CLAMP_ON_RETRACT, robot));
            outtakeActions.add(new MotionAction(robot.getOuttakeExtender(), robot.OUTTAKE_SLIDE_RETRACTED, currentTime + robot.DELAY_SLIDE_ON_RETRACT, robot));
        } else if (!gamepad2.b) {
            isTogglingG2B = false;
        }
        if (gamepad2.x && !isTogglingG2X) {
            isTogglingG2X = true;

            outtakeActions.clear();

            outtakeActions.add(new MotionAction(robot.getIntakePusher(), robot.PUSHER_PUSHED, currentTime + robot.DELAY_PUSHER_ON_CLAMP, robot));
            outtakeActions.add(new MotionAction(robot.getIntakePusher(), robot.PUSHER_RETRACTED, currentTime + robot.DELAY_RETRACT_PUSHER_ON_CLAMP, robot));
            outtakeActions.add(new MotionAction(robot.getFrontClamp(), robot.FRONTCLAMP_CLAMPED, currentTime + robot.DELAY_CLAMP_ON_CLAMP, robot));
            outtakeActions.add(new MotionAction(robot.getBackClamp(), robot.BACKCLAMP_CLAMPED, currentTime + robot.DELAY_CLAMP_ON_CLAMP, robot));
        } else if (!gamepad2.x) {
            isTogglingG2X = false;
        }

        boolean motionExecuted = true;
        while (motionExecuted) {
            currentTime = SystemClock.elapsedRealtime();
            MotionAction currentMotion = outtakeActions.peek();

            if (currentMotion != null) {
                if (currentMotion.getDelayStartTime() <= currentTime) {
                    currentMotion.executeMotion();
                    outtakeActions.remove();
                } else {
                    motionExecuted = false;
                }
            } else {
                motionExecuted = false;
            }
        }
    }

    private void foundationLogic() {
        if (gamepad1.right_bumper) {
            if (foundationToggle && !resetfoundation) {
                foundationToggle = false;
            } else if (!foundationToggle && !resetfoundation) {
                foundationToggle = true;
            }
            resetfoundation = true;
        } else {
            resetfoundation = false;
        }

        robot.foundationMovers(foundationToggle);
    }

    private boolean toggleMode = true;

    private void robotModeLogic() {
        if (gamepad1.a && gamepad1.x && toggleMode) {
            if (isIntakeMode) {
                isIntakeMode = false;
            } else {
                isIntakeMode = true;
            }
            toggleMode = false;
        } else if (!toggleMode && !(gamepad1.a && gamepad1.x)) {
            toggleMode = true;
        }
    }
}