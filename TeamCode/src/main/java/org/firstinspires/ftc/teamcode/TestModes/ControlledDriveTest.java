package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;


@TeleOp(name = "ControlledDriveTest")

public class ControlledDriveTest extends OpMode {
    ControlledDrive controlledDrive;
    HardwareChassis robot;

    @Override
    public void init() {
        this.controlledDrive = new ControlledDrive(hardwareMap, telemetry, () -> true);
        this.robot = new HardwareChassis(hardwareMap);
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


