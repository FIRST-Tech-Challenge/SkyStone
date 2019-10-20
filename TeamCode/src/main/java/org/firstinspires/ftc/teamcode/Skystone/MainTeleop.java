package org.firstinspires.ftc.teamcode.Skystone;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;

import java.security.Policy;

@TeleOp(name="MainTeleOpSky", group="Linear Opmode")
public class MainTeleop extends LinearOpMode {
    Robot robot;

    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    double intakeLeftPower;
    double intakeRightPower;

    char outtakeButton;

    long outtakeExecutionTime;

    long currentTime;

    double outtakePivotExecutePosition;
    double outtakePivotWaitTime;
    double outtakeExtenderExecutePosition;
    double outtakeExtenderWaitTime;
    double outtakeClampExecutePosition;
    double outtakeClampWaitTime;

    boolean onSlowDrive, changedSlowDrive = false;

    public static double powerScaleFactor = 0.9;

    @Override
    public void runOpMode() {
        resetRobot();
        waitForStart();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
        while (opModeIsActive()) {
            slowDriveLogic();
            driveLogic();
            outtakeLogic();
            intakeLogic();
            if(gamepad2.dpad_up){
                robot.outtakeSpool.setPower(1);
            }else if(gamepad2.dpad_down){
                robot.outtakeSpool.setPower(-1);
            }else{
                robot.outtakeSpool.setPower(0);
            }
//            outtakeLogic();
        }
    }

    private void resetRobot(){
        robot = new Robot(hardwareMap, telemetry, this);
        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.outtakeSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        robot.outtakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.outtakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //teleop methods
    private void driveLogic(){
        //tank drive
        fLPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x)*powerScaleFactor;
        fRPower = (-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x)*powerScaleFactor;
        bLPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x)*powerScaleFactor;
        bRPower = (-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x)*powerScaleFactor;

        if(gamepad1.right_trigger!=0){
            fLPower = (gamepad1.right_trigger)*powerScaleFactor;
            fRPower = (-gamepad1.right_trigger)*powerScaleFactor;
            bLPower = (-gamepad1.right_trigger)*powerScaleFactor;
            bRPower = (gamepad1.right_trigger)*powerScaleFactor;
        }else if(gamepad1.left_trigger!=0){
            fLPower = (-gamepad1.left_trigger)*powerScaleFactor;
            fRPower = (gamepad1.left_trigger)*powerScaleFactor;
            bLPower = (gamepad1.left_trigger)*powerScaleFactor;
            bRPower = (-gamepad1.left_trigger)*powerScaleFactor;
        }
        //Straight D-Pad move
        if (gamepad1.dpad_up) {
            fLPower = (gamepad1.left_stick_y)+powerScaleFactor;
            bLPower = (gamepad1.left_stick_y)+powerScaleFactor;
            fRPower = (gamepad1.right_stick_y+powerScaleFactor);
            bRPower = (gamepad1.right_stick_y+powerScaleFactor);
        } else if (gamepad1.dpad_down) {
            fLPower = (gamepad1.left_stick_y)-powerScaleFactor;
            bLPower = (gamepad1.left_stick_y)-powerScaleFactor;
            fRPower = (gamepad1.right_stick_y-powerScaleFactor);
            bRPower = (gamepad1.right_stick_y)-powerScaleFactor;
        } else if (gamepad1.dpad_right) {
            fLPower = (gamepad1.right_stick_y)+powerScaleFactor;
            bLPower = (gamepad1.right_stick_y)+powerScaleFactor;
            fRPower = (gamepad1.left_stick_y)-powerScaleFactor;
            bRPower = (gamepad1.left_stick_y)-powerScaleFactor;
        } else if (gamepad1.dpad_left) {
            fRPower = (gamepad1.right_stick_y)+powerScaleFactor;
            bRPower = (gamepad1.right_stick_y)+powerScaleFactor;
            fLPower = (gamepad1.left_stick_y)-powerScaleFactor;
            bLPower = (gamepad1.left_stick_y)-powerScaleFactor;
        }

        robot.allWheelDrive(fLPower, fRPower, bLPower, bRPower);
    }

    private void slowDriveLogic(){
        //toggle driving speed
        if (powerScaleFactor == 0.3) {
//            telemetry.addData("Driving Mode","Slow");
        } else {
//            telemetry.addData("Driving Mode","Normal");
        }
        if (gamepad1.left_bumper && !changedSlowDrive) {
            powerScaleFactor = (onSlowDrive) ? 0.9 : 0.3;
            onSlowDrive = !onSlowDrive;
            changedSlowDrive = true;
        } else if(!gamepad1.left_bumper) {
            changedSlowDrive = false;
        }
    }

    private void intakeLogic() {
        robot.intakeLeft.setPower(gamepad2.left_stick_y);
        robot.intakeRight.setPower(gamepad2.right_stick_y);
    }

    private void outtakeLogic() {
        currentTime = SystemClock.elapsedRealtime();

        // Logic to control outtake; with a delay on the pivot so that the slides can extend before pivot rotation
        if (gamepad2.a) { // Clamp and Extend
            outtakeExecutionTime = currentTime;

            outtakeClampExecutePosition = robot.CLAW_SERVO_CLAMPED;
            outtakeClampWaitTime = 0;

            outtakeExtenderExecutePosition = robot.OUTTAKE_PIVOT_EXTENDED;
            outtakeExtenderWaitTime = 1000;

            outtakePivotExecutePosition = robot.OUTTAKE_PIVOT_EXTENDED;
            outtakePivotWaitTime = 2000;
        } else if (gamepad2.b) { // Deposit and Reset
            outtakeExecutionTime = currentTime;

            outtakeClampExecutePosition = robot.CLAW_SERVO_RELEASED;
            outtakeClampWaitTime = 0;

            // No need for delay in pivot because slides are already extended
            outtakePivotExecutePosition = robot.OUTTAKE_PIVOT_RETRACTED;
            outtakePivotWaitTime = 0;

            outtakeExtenderExecutePosition = robot.OUTTAKE_SLIDE_RETRACTED;
            outtakeExtenderWaitTime = 2000;
        } else if (gamepad2.x) { // Clamp
            robot.clamp.setPosition(robot.CLAW_SERVO_CLAMPED);
        } else if (gamepad2.y) { // Extend
            outtakeExecutionTime = currentTime;

            outtakeExtenderExecutePosition = robot.OUTTAKE_PIVOT_EXTENDED;
            outtakeExtenderWaitTime = 0;

            outtakePivotExecutePosition = robot.OUTTAKE_PIVOT_EXTENDED;
            outtakePivotWaitTime = 1000;
        }

        // Execute pivot rotation after wait time
        if (SystemClock.elapsedRealtime() == outtakeExecutionTime + outtakeClampWaitTime) {
            robot.clamp.setPosition(outtakeClampExecutePosition);
        }

        if (SystemClock.elapsedRealtime() == outtakeExecutionTime + outtakePivotWaitTime) {
            robot.clampPivot.setPosition(outtakePivotExecutePosition);
        }

        if (SystemClock.elapsedRealtime() == outtakeExecutionTime + outtakeExtenderWaitTime) {
            robot.outtakeExtender.setPosition(outtakeExtenderExecutePosition);
        }

        // Spool controls
        if (gamepad2.dpad_up) {
            robot.outtakeSpool.setPower(.5);
        } else if (gamepad2.dpad_down) {
            robot.outtakeSpool.setPower(-.5);
        } else {
            robot.outtakeSpool.setPower(0);
        }
    }
}


