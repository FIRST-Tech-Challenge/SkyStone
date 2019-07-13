package com.hfrobots.tnt.season1819.support;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.FeedforwardTuningOpMode;
import com.hfrobots.tnt.season1819.RoadrunnerMecanumDriveAdapter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Feedforward Tuner", group="Utilities")
@Disabled
public class TntFeedforwardTuningOpMode extends FeedforwardTuningOpMode {
    public TntFeedforwardTuningOpMode() {
        // TODO: change the following to match your drive
        super(7*12, RoadrunnerMecanumDriveAdapter.MOTOR_MAX_RPM, 4.0);
    }

    @Override
    protected Drive initDrive() {
        return new RoadrunnerMecanumDriveAdapter(hardwareMap);
    }
}