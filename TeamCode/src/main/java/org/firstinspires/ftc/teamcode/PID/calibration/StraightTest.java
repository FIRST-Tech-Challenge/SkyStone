package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.All.HardwareMap;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.PID.mecanum.SampleMecanumDriveREV;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "StraightTest", group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareMap map = new HardwareMap(hardwareMap);

        map.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        map.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        map.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        map.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(150);

        map.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        map.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        map.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        map.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        //HardwareMap.track.resetEncoders();
        //HardwareMap.track.encoders(true);

        telemetry.addData("STATUS", "Ready for START!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Thread followTraj = new Thread() {
            public void run() {
                drive.followTrajectorySync(trajectory);
            }
        };
        followTraj.start();

        while(opModeIsActive()){
            telemetry.addData("LeftForward", map.leftIntake.getCurrentPosition());
            telemetry.addData("RightForward", map.liftTwo.getCurrentPosition());
            telemetry.addData("Sideways", map.rightIntake.getCurrentPosition());
            telemetry.update();

            /*telemetry.addData("LeftForward", HardwareMap.track.getEncoderTicks().get(0));
            telemetry.addData("RightForward", HardwareMap.track.getEncoderTicks().get(1));
            telemetry.addData("Sideways", HardwareMap.track.getEncoderTicks().get(2));

            telemetry.addData("Formula", HardwareMap.track.getEncoderDebug().get(0));
            telemetry.addData("LeftDebug", HardwareMap.track.getEncoderDebug().get(1));
            telemetry.addData("RightDebug", HardwareMap.track.getEncoderDebug().get(2));
            telemetry.addData("SidewaysDebug", HardwareMap.track.getEncoderDebug().get(3));

            telemetry.update();*/
        }
    }
}
