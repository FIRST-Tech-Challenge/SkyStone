package com.hfrobots.tnt.season1819.support;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.TrackWidthCalibrationOpMode;
import com.hfrobots.tnt.season1819.RoadrunnerMecanumDriveAdapter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Trackwidth Tuner", group="Utilities")
@Disabled
public class TntTrackWidthCalibrationOpMode extends TrackWidthCalibrationOpMode {
    @Override
    protected Drive initDrive() {
        return new RoadrunnerMecanumDriveAdapter(hardwareMap);
    }

    @Override
    protected BNO055IMU initIMU() {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        return imu;
    }
}
