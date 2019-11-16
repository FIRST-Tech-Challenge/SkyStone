package org.firstinspires.ftc.teamcode.PID.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        drive.followTrajectorySync(trajectory);

        while(opModeIsActive()){
            telemetry.addData("LeftForward", map.frontRight.getCurrentPosition());
            telemetry.addData("RightForward", map.backLeft.getCurrentPosition());
            telemetry.addData("Sideways", map.backRight.getCurrentPosition());

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
