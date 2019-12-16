package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledDriveOld;


@TeleOp(name = "ControlledDriveTest")

public class ControlledDriveTest extends OpMode {
    HardwareChassis robot;

    @Override
    public void init() {
        this.robot = new HardwareChassis(hardwareMap);
    }

    @Override
    public void loop() {
        double speed = 0.2;
        double distance = 50;
        int timeout = 2; // not working yet

        //controlledDriveOld.driveDistance(distance, 0, speed, timeout);
        //controlledDriveOld.driveDistance(0, distance, speed, timeout);
        //controlledDriveOld.driveDistance(-distance, 0, speed, timeout);
        //controlledDriveOld.driveDistance(0, -distance, speed, timeout);
    }
}


