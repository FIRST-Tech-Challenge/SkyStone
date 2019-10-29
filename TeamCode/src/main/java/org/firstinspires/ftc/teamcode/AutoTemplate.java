package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
//@Disabled
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
        while (opModeIsActive())
        {
            detector.updateVals();
            vals = detector.getVals();
            telemetry.addData("Values", vals[1]+"   "+vals[0]+"   "+vals[2]);

            telemetry.addData("Global heading", robot.getAngle());
            telemetry.update();
            telemetry.update();
            sleep(100);
        }
    }
}
