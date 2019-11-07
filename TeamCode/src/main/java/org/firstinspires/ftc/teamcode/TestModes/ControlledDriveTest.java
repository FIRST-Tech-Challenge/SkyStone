package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareOmniTest;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;

@Autonomous(name = "ControlledDriveTest")

public class ControlledDriveTest extends OpMode {
    ControlledDrive controlledDrive;
    HardwareOmniTest robot;

    @Override
    public void init() {
        this.controlledDrive = new ControlledDrive(hardwareMap, telemetry);
        this.robot = new HardwareOmniTest(hardwareMap);
    }

    @Override
    public void loop() {
        double speed = 0.2;
        double distance = 50;
        int timeout = 2; // not working yet
        controlledDrive.driveDistance(distance, 0, speed, timeout);
        controlledDrive.driveDistance(0, distance, speed, timeout);
        controlledDrive.driveDistance(-distance, 0, speed, timeout);
        controlledDrive.driveDistance(0, -distance, speed, timeout);
    }
}
