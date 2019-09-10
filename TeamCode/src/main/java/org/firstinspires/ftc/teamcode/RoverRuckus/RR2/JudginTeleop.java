package org.firstinspires.ftc.teamcode.RoverRuckus.RR2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Deprecated
@Disabled
@TeleOp(name="Judging Teleop", group="Linear Opmode")
public class JudginTeleop extends LinearOpMode {
    RR2 robot;
    boolean isRightStick = false;
    boolean facts = false;

    double sign = 1;
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    double slidePower;

    boolean isHangStarted = false;
    boolean killSwitchForHangTouchIsHit = false;

    double intakePower;
    boolean onSlowDrive, changedSlowDrive = false;

    public static double powerScaleFactor = 1;

    @Override
    public void runOpMode() {
        resetRobot();
        waitForStart();
        robot.hangLockOpen();
        while (opModeIsActive()) {
            slowDriveLogic();
            driveLogic();
            intakeLogic();
            blockerLogic();
            slideLogic();
            pivotLogic();
            hangLockLogic();
            manualLatchLogic();
            setToHangMode();
            hangRobot();
            dropRobot();
            hangTouchKillSwitch();
        }
    }

    private void resetRobot(){
        robot = new RR2(hardwareMap, telemetry, this);

        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetMotor(robot.pivot);
        robot.resetMotor(robot.pivot2);
        robot.resetMotor(robot.slide);

        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    private void intakeLogic(){
        //Intake Control
        intakePower = -gamepad2.left_stick_y;
    }

    private void blockerLogic(){
        //blocker for outtake
        if(gamepad2.right_bumper){
            robot.blocker.setPosition(0.325);
            intakePower = -1;
        }else{
            robot.blocker.setPosition(0.7);
        }
        robot.intake.setPower(intakePower);
    }

    private void pivotLogic(){
        //Pivoting Slide For Outtake
        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine(Integer.toString(robot.pivot.getCurrentPosition()));
        telemetry.addLine(Double.toString(robot.pivot.getPower()));
        telemetry.update();

        if(robot.pivot.getCurrentPosition()<=-2000 && !facts){
            sign*=-1;
            facts = true;
        }

        if(robot.pivot.getCurrentPosition()>=-2000 && facts){
            sign*=-1;
            facts = false;
        }

        if(facts && !isRightStick && gamepad2.right_stick_y==0){
            slidePower = (robot.pivot.getPower()*sign)/1.05;
        }

        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (gamepad1.y) {
            robot.pivot.setPower(0.9);
            robot.pivot2.setPower(-0.9);
        } else if (gamepad1.x && robot.distance.getDistance(DistanceUnit.MM)>150) {
            robot.pivot.setPower(-0.8);
            robot.pivot2.setPower(0.8);
        } else {
            robot.pivot.setPower(0);
            robot.pivot2.setPower(0);
        }

        robot.slide.setPower(slidePower);
    }

    private void slideLogic(){
        slidePower = gamepad2.right_stick_y;
    }

    private void hangLockLogic(){
        //Hang Locking
        if (gamepad2.left_bumper) {
            robot.hangLockClose();
        }
        else if (gamepad2.right_bumper) {
            robot.hangLockOpen();
        }
    }

    private void manualLatchLogic(){
        //manual control of the latch
        if (gamepad2.a) {
            robot.hook.setPosition(0);
        }else if (gamepad2.b) {
            robot.hook.setPosition(1);
        }
    }

    private void slowDriveLogic(){
        //toggle driving speed
        if(powerScaleFactor == 0.3){
            telemetry.addData("Driving Mode","Slow");
        }else{
            telemetry.addData("Driving Mode","Normal");
        }
        if(gamepad1.left_bumper && !changedSlowDrive){
            powerScaleFactor = (onSlowDrive) ? 0.8 : 0.3;
            onSlowDrive = !onSlowDrive;
            changedSlowDrive = true;
        }else if(!gamepad1.left_bumper){
            changedSlowDrive = false;
        }
        telemetry.update();
    }

    private void setToHangMode(){
        //prepares robot to hang - opens latch and moves the pivot into position
        if(gamepad1.a) {
            isHangStarted = true;
            robot.hangLockOpen();
            robot.hook.setPosition(0);
            telemetry.addData("hook status","opening...");
            telemetry.update();

            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("pivot before",robot.pivot.getCurrentPosition());
            telemetry.addData("pivot2 before",robot.pivot2.getCurrentPosition());
            telemetry.update();
            robot.pivot.setPower(1);//this
            robot.pivot2.setPower(-1);

            while(robot.pivot.getCurrentPosition() > -4400 && opModeIsActive()){
                telemetry.addData("pivot",robot.pivot.getCurrentPosition());
                telemetry.addData("pivot2",robot.pivot2.getCurrentPosition());
                telemetry.update();
                driveLogic();
            }
            telemetry.addData("pivot after",robot.pivot.getCurrentPosition());
            telemetry.addData("pivot2 after",robot.pivot2.getCurrentPosition());
            telemetry.update();
            robot.pivot.setPower(0);
            robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.pivot2.setPower(0);
            robot.pivot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void hangTouchKillSwitch(){
        if(gamepad1.left_stick_button){
            killSwitchForHangTouchIsHit = true;
        }
    }

    private void hangRobot(){
        //hangs the robot by pulling it up
        if(gamepad1.b) {
//            robot.hook.setPosition(1);
//            sleep(1500);
            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivot.setPower(-1);
            robot.pivot2.setPower(1);
            while(robot.distance.getDistance(DistanceUnit.MM)>170 && opModeIsActive()){
                if(gamepad2.right_bumper){
                    telemetry.addData("Hang Status","Aborting hang...");
                    telemetry.update();
                    return;
                }else{
                    telemetry.addData("Hang Status","Hanging...");
                    telemetry.update();
                }
            }
            sleep(500);
            robot.pivot.setPower(0);
            robot.pivot2.setPower(0);
            robot.hangLockClose();
            telemetry.addData("Hang Status","Hang successful");
            telemetry.update();
        }
    }

    private void dropRobot(){
        //drop robot
        if(gamepad1.left_bumper && gamepad1.right_bumper){
            robot.pivot.setPower(1);
            while(robot.distance.getDistance(DistanceUnit.MM)>150 && opModeIsActive()){
                telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            telemetry.addData("done", "done");
            robot.pivot.setPower(0);

            robot.hangLockOpen();
            sleep(1000);

            robot.pivot.setPower(-1);

            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pivot.setTargetPosition(-4000);

            while(robot.pivot.isBusy() && opModeIsActive()){
                telemetry.addData("encoder value of Pivot", robot.pivot.getCurrentPosition());
                telemetry.update();
            }
            telemetry.update();
            robot.pivot.setPower(0);

            robot.hook.setPosition(0); //open

            robot.pivot.setPower(1);
            robot.pivot.setTargetPosition(0);
        }
    }
}



