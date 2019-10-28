package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import teamcode.common.StandardDriveSystem;
import teamcode.common.TTOpMode;

@Autonomous(name = "Simple Parking Auto")
public class SimpleParkingAuto extends TTOpMode {

    private StandardDriveSystem driveSystem;

    @Override
    protected void onInitialize() {
        driveSystem = new StandardDriveSystem(hardwareMap);
    }

    @Override
    protected void onStart() {

    }

    @Override
    protected void onStop() {

    }

}
