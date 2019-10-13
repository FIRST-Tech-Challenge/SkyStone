package org.firstinspires.ftc.teamcode.supportops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libraries.AutoLib2019;

@Autonomous(group = "Support")
public class TestVuforia extends LinearOpMode {
    private AutoLib2019 autoLib;

    @SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("about to move", "initialized");
        telemetry.update();
        String stonePosition = autoLib.findSkyStone();
        telemetry.addData("Status", "Found Skystone");
        telemetry.update();
        telemetry.addData("Status", stonePosition);
        telemetry.update();
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        autoLib = new AutoLib2019(this);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }


}