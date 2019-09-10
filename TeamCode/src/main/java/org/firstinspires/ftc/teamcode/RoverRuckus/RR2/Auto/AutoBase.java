package org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.RR2;
@Deprecated
public class AutoBase extends LinearOpMode {

    public TensorFlowMineralDetection tensorFlowMineralDetection;
    public RR2 robot;

    public void initLogic(){
        //Init's robot
        tensorFlowMineralDetection = new TensorFlowMineralDetection(hardwareMap,telemetry,this);
        robot = new RR2(hardwareMap,telemetry,this);

        robot.setBrakeModeDriveMotors();

        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.blocker.setPosition(0.7);

        tensorFlowMineralDetection.initVuforia();
        tensorFlowMineralDetection.initTfod();

        waitForStart();
    }

    @Override
    public void runOpMode(){

    }

    protected void objectDetection(){
        telemetry.addLine(tensorFlowMineralDetection.runObjectDetection().toString());
        telemetry.update();
    }

    protected void dropDownFromLander() {
        objectDetection();
        robot.pivot.setPower(-1);
        robot.pivot2.setPower(1);
        robot.resetEncoders();
        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Move robot to relase hanglocks
        long startTime = SystemClock.elapsedRealtime();
        while (robot.distance.getDistance(DistanceUnit.MM) > 170 && opModeIsActive() &&
                SystemClock.elapsedRealtime()-startTime<1500 ) {
            telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        telemetry.addData("done", "done");
        robot.pivot.setPower(0);
        robot.pivot2.setPower(0);

        robot.hangLockOpen();
        sleep(1000);

        robot.pivot.setPower(1);
        robot.pivot2.setPower(-1);
        //Brings Robot Down From Lander
        startTime = SystemClock.elapsedRealtime();
        while (robot.bottomDistance.getDistance(DistanceUnit.MM) > 23 &&
                opModeIsActive() && robot.pivot.getCurrentPosition() > -4550 && SystemClock.elapsedRealtime()-startTime<4000) {
            telemetry.addData("encoder value of Pivot", robot.pivot.getCurrentPosition());
            telemetry.update();
        }

        telemetry.update();
        robot.pivot.setPower(0);
        robot.pivot2.setPower(0);

        robot.hook.setPosition(0); //open
        robot.intializeIMU();

        robot.pivot.setPower(-1);
        robot.pivot2.setPower(1);
        startTime = SystemClock.elapsedRealtime();
        //Pivots down after unhook
        while (robot.distance.getDistance(DistanceUnit.MM) > 170 && opModeIsActive() && robot.pivot.getCurrentPosition() < -200 && SystemClock.elapsedRealtime()-startTime<2000) {
            telemetry.addData("encoder value of Pivot", robot.distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

        robot.pivot2.setPower(0);
        robot.pivot.setPower(0);
    }

    protected void knockOffMineral(double leftRightAngle) {
        if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.RIGHT){
            robot.finalTurn(-leftRightAngle);
            robot.finalMove(0.75, 58);
        }else if(tensorFlowMineralDetection.location == TensorFlowMineralDetection.Location.LEFT){
            robot.finalTurn(leftRightAngle);
            robot.finalMove(0.75, 58);
        } else {
            robot.finalMove(0.75, 53);
        }
    }
}
