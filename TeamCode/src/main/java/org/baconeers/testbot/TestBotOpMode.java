package org.baconeers.testbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.baconeers.common.BaconOpMode;
import org.baconeers.common.GamePadSteerDrive;

@TeleOp(name = "TestBotOpMode")
public class TestBotOpMode extends BaconOpMode {
    private TestBotConfiguration config;
    private GamePadSteerDrive drive;

    @Override
    protected void onInit() {
       config  = TestBotConfiguration.newConfig(hardwareMap, telemetry);

       drive = new GamePadSteerDrive(this, gamepad1, config.leftMotor, config.rightMotor);

    }

    @Override
    protected void activeLoop() throws InterruptedException {
        drive.update();

        if (config.touchSensor.isPressed()) {
            telemetry.addLine("Touch pressed");
        }
    }
}
