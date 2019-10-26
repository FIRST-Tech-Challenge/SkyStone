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

    boolean isRetract = false;
    boolean isExtend = false;

    public static double powerScaleFactor = 0.9;

    @Override
    public void runOpMode() {
        resetRobot();
        initServos();
        waitForStart();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
        while (opModeIsActive()) {
            slowDriveLogic();
            driveLogic();
            outtakeLogic2();
            intakeLogic();
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

    private void initServos() {
        robot.clampPivot.setPosition(robot.OUTTAKE_PIVOT_RETRACTED);
        robot.intakePusher.setPosition(0.7);
        robot.outtakeExtender.setPosition(robot.OUTTAKE_SLIDE_RETRACTED);
        robot.clamp.setPosition(robot.CLAW_SERVO_CLAMPED);
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

        if (gamepad2.left_stick_y > 0 && gamepad2.right_stick_y > 0) {
            robot.intakePusher.setPosition(robot.PUSHER_RETRACTED);
        }
    }

    private void outtakeLogic2() {
        currentTime = SystemClock.elapsedRealtime();

        // Logic to control outtake; with a delay on the pivot so that the slides can extend before pivot rotation
        if (gamepad2.a) { // Clamp and Extend
            robot.intakePusher.setPosition(robot.PUSHER_PUSHED); // Push block all the way to clamp

            isExtend = true;
            isRetract = false;

            outtakeExecutionTime = currentTime;
        } else if (gamepad2.b) { // Deposit and Reset
            robot.intakePusher.setPosition(robot.PUSHER_RETRACTED); // Reset intake pusher

            isRetract = true;
            isExtend = false;

            outtakeExecutionTime = currentTime;

            robot.clamp.setPosition(robot.CLAW_SERVO_RELEASED); // Release clamp
        }

        //extend
        if (currentTime - outtakeExecutionTime >= 200 && isExtend) {
            robot.clamp.setPosition(robot.CLAW_SERVO_CLAMPED);
        }

        if(currentTime-outtakeExecutionTime >= 300 && isExtend){
            robot.outtakeExtender.setPosition(robot.OUTTAKE_SLIDE_EXTENDED);
        }

        if(currentTime-outtakeExecutionTime >= 1200 && isExtend){
            robot.clampPivot.setPosition(robot.OUTTAKE_PIVOT_EXTENDED);
            isExtend = false;
        }

        //retract
        if(currentTime-outtakeExecutionTime >= 250 && isRetract){
            robot.clampPivot.setPosition(robot.OUTTAKE_PIVOT_RETRACTED);
        }

        if(currentTime-outtakeExecutionTime >= 750 && isRetract){
            robot.outtakeExtender.setPosition(robot.OUTTAKE_SLIDE_RETRACTED);
            isRetract = false;
        }

        // Spool controls
        if (gamepad2.dpad_up) {
            robot.outtakeSpool.setPower(1);
        } else if (gamepad2.dpad_down) {
            robot.outtakeSpool.setPower(-.1);
        } else {
            robot.outtakeSpool.setPower(0);
        }
    }
}