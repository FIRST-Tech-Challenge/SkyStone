package org.firstinspires.ftc.teamcode.Skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;

@TeleOp(name="MainTeleOpSky", group="Linear Opmode")
public class MainTeleop extends LinearOpMode {
    Robot robot;

    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

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
        }
    }

    private void resetRobot(){
        robot = new Robot(hardwareMap, telemetry, this);
        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if(powerScaleFactor == 0.3){
//            telemetry.addData("Driving Mode","Slow");
        }else{
//            telemetry.addData("Driving Mode","Normal");
        }
        if(gamepad1.left_bumper && !changedSlowDrive){
            powerScaleFactor = (onSlowDrive) ? 0.9 : 0.3;
            onSlowDrive = !onSlowDrive;
            changedSlowDrive = true;
        }else if(!gamepad1.left_bumper){
            changedSlowDrive = false;
        }
    }
}



