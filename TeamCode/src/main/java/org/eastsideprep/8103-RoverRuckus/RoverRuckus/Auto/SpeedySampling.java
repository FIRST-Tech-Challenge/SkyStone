package org.firstinspires.ftc.teamcode.RoverRuckus.Auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveSystems.Mecanum.RoadRunner.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.Mechanisms.SparkyTheRobot;
import org.firstinspires.ftc.teamcode.RoverRuckus.Deployers.Auto.EndGoal;
import org.firstinspires.ftc.teamcode.RoverRuckus.Deployers.Auto.StartingPosition;
import org.firstinspires.ftc.teamcode.Utilities.RoadRunner.AssetsTrajectoryLoader;

import java.io.IOException;

@Config
@Autonomous(name="Speedy double sampling")
public abstract class SpeedySampling extends SuperiorSampling {

    @Override
    public void runOpMode() throws InterruptedException {
        // Set up road runner
        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(hardwareMap);

        robot = new SparkyTheRobot(this);
        robot.calibrate(true);
        robot.markerDeployer.setPosition(MARKER_DEPLOYER_RETRACTED);
        robot.parkingMarker.setPosition(PARKING_MARKER_RETRACTED);
        initVuforia();
        setWinchHoldPosition();

        // Display telemetry feedback
        telemetry.log().add("Running speedy sampling op-mode");
        telemetry.log().add("Starting from: [[" + startingPosition.name() + "]]");
        telemetry.log().add("Final actions: [[" + goal.name() + "]]");
        telemetry.update();

        GoldPosition goldLoc = waitAndWatchMinerals();
        ElapsedTime timeSinceStart = new ElapsedTime();

        if (isStopRequested()) return;

        if (position == 0) {
            goldLoc = GoldPosition.LEFT;
        } else if (position == 1) {
            goldLoc = GoldPosition.CENTER;
        } else if (position == 2) {
            goldLoc = GoldPosition.RIGHT;
        }

        // Use appropriate method for dehooking
        double[] possibleHeadings = {Math.PI * 0.5, Math.PI * 0.75, Math.PI};
        double[] unhookDelay = {3.49, 3.92, 4.58};
        unhookFromLander(drive, robot, possibleHeadings[goldLoc.index]);
        Log.i("PreDriveHeading",robot.getGyroHeading() + "");

        new java.util.Timer().schedule(
                new java.util.TimerTask() {
                    @Override
                    public void run() {
                        robot.markerDeployer.setPosition(MARKER_DEPLOYER_DEPLOY);
                        robot.parkingMarker.setPosition(PARKING_MARKER_EXTENDED);
                        beginAppendageSwap();
                    }
                }, (int) unhookDelay[goldLoc.index] * 1000);
        new java.util.Timer().schedule(
                new java.util.TimerTask() {
                    @Override
                    public void run() {
                        robot.intake.collect();
                    }
                }, (int) (unhookDelay[goldLoc.index] + 1) * 1000);

        try {
            followPath(drive, AssetsTrajectoryLoader.load("Depo" + goldLoc.fileName + "SelFast"));
            robot.parkingMarker.setPosition(PARKING_MARKER_EXTENDED);
            while (robot.hangSwitch.getState() && opModeIsActive()) {}
            endAppendageSwap();
        } catch (IOException e) {
            // TODO retry load
            e.printStackTrace();
        }
    }
}
