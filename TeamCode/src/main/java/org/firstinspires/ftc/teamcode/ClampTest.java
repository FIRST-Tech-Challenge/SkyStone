package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.revextensions2.ExpansionHubServo;

import java.io.File;

/**
 * 2019.10.26
 * Created by Ian Q.
 */
@TeleOp(name = "ClampTest", group = "Test")
public class ClampTest extends OpMode {
    RobotHardware robotHardware;
    RobotProfile robotProfile;

    ExpansionHubServo blockHolderServo1, blockHolderServo2;

    @Override
    public void init() {
        try {
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }

        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap, robotProfile);

        blockHolderServo1 = (ExpansionHubServo) hardwareMap.servo.get("BlockHolderServo1");
        blockHolderServo2 = (ExpansionHubServo) hardwareMap.servo.get("BlockHolderServo2");

        blockHolderServo1.setPosition(0.3);
        blockHolderServo2.setPosition(robotProfile.hardwareSpec.clampS2Close);

        telemetry.addData("Block Holder Servo 1", blockHolderServo1.getPosition());
        telemetry.addData("Block Holder Servo 2", blockHolderServo2.getPosition());
    }

    @Override
    public void loop() {
    }
}
