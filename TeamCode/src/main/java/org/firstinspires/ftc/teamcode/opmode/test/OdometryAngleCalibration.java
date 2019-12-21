package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

@Config
@Autonomous(name="Odometry Angle Calibration")
public class OdometryAngleCalibration extends LinearOpMode {

    public static int amountOfTrials = 5;
    public static double targetAngle = Math.PI;
    public static double globalAngle = 0;
    public static double lastAngle = 0;

    private SampleMecanumDriveREVOptimized drive;

    public void runOpMode(){

        drive = SampleMecanumDriveREVOptimized.getInstance(hardwareMap);
        waitForStart();

        for(int i = 0; i < amountOfTrials; i++){
            if(isStopRequested()){
                break;
            }

            drive.turn(targetAngle);
            while(drive.isBusy() && !isStopRequested()){
                drive.update();
                telemetry.addData("Lateral Distance: ", StandardTrackingWheelLocalizer.LATERAL_DISTANCE);
                telemetry.addData("Reported Angle: ", drive.getPoseEstimate().getHeading());
                telemetry.addData("IMU Angle: ", globalAngle);
                telemetry.update();
            }

            double deltaAngle = drive.getExternalHeading() - lastAngle;

            if (deltaAngle < -180) {
                deltaAngle += 360;
            } else if (deltaAngle > 180) {
                deltaAngle -= 360;
            }

            globalAngle += deltaAngle;

            lastAngle = drive.getExternalHeading();

            double newLateralDistance = (StandardTrackingWheelLocalizer.LATERAL_DISTANCE * drive.getPoseEstimate().getHeading()) / (globalAngle);
            StandardTrackingWheelLocalizer.LATERAL_DISTANCE = newLateralDistance;
            telemetry.addData("New Lateral Distance: ", newLateralDistance);
            telemetry.update();
            resetAngles();
        }
    }

    public void resetAngles(){
        drive.setPoseEstimate(new Pose2d(0,0,0));
        globalAngle = 0;
    }
}
