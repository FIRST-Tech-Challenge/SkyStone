package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.TTDriveSystem;
import teamcode.common.TTOpMode;

@Autonomous(name = "Drive Callibration")
public class DriveCallibration extends TTOpMode {

    TTDriveSystem driveSystem;

    @Override
    protected void onInitialize() {
        driveSystem = new TTDriveSystem(hardwareMap);
    }

    @Override
    protected void onStart() {
        driveSystem.vertical(50.0, 0.5);
    }

    @Override
    protected void onStop() {

    }

}
