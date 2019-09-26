package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDrive;

public class ControlledDriveTest extends OpMode {
    ControlledDrive controlledDrive

    @Override
    public void init() {
        this.controlledDrive = new ControlledDrive(hardwareMap);
    }

    @Override
    public void loop() {
        controlledDrive.driveDistance(20, 0);
        controlledDrive.driveDistance(0, 20);
        controlledDrive.driveDistance(-20, 0);
        controlledDrive.driveDistance(0, -20);
    }
}
