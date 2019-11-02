package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileReader;

import pkg3939.Robot3939;
import pkg3939.skystoneDetectorClass;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 */
@Autonomous(name= "AutoTemplate", group="Sky autonomous")
@Disabled
public class AutoTemplate extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Robot3939 robot = new Robot3939();
    skystoneDetectorClass detector = new skystoneDetectorClass();
    int[] vals;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initMotors(hardwareMap);//motors
        robot.initServos(hardwareMap);//servo
        robot.initIMU(hardwareMap);//gyro

        detector.setOffset(0, 0);//
        detector.camSetup(hardwareMap);

        robot.useEncoders(false);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();
        runtime.reset();
            detector.updateVals();
            vals = detector.getVals();

            robot.FL.setPower(0.3);
            robot.FR.setPower(0.3);
            robot.RL.setPower(0.3);
            robot.RR.setPower(0.3);
            sleep(1500);
            telemetry.addData("haha i suck", "ea");
            telemetry.update();

            robot.FL.setPower(-0.6);
            robot.FR.setPower(-0.6);
            robot.RL.setPower(-0.6);
            robot.RR.setPower(-0.6);
            telemetry.addData("fu", "hk");
            telemetry.update();

            telemetry.addData("Values", vals[1]+"   "+vals[0]+"   "+vals[2]);
            telemetry.addData("Global heading", robot.getAngle());
            telemetry.update();


    }

    public void moveDistance(double power, double distance) {
        robot.RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double distancePerRotation = 3.1415 * 4; //pi * diameter (inches)
        double rotations = -distance/distancePerRotation; //distance / circumference (inches)
        int encoderDrivingTarget = (int)(rotations*1120);

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(encoderDrivingTarget);
            robot.RR.setTargetPosition(encoderDrivingTarget);
//            robot.FL.setTargetPosition(encoderDrivingTarget);
//            robot.FR.setTargetPosition(encoderDrivingTarget);

            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 //           robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.RL.setPower(power);
            robot.RR.setPower(power);
// || robot.FL.isBusy() || robot.FR.isBusy()


            while(robot.RL.isBusy() || robot.RR.isBusy()) {
                //wait till motor finishes working
                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.update();
            }

            robot.RL.setPower(0);
            robot.RR.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
