package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareClampTest;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledLift;

@TeleOp(name = "ClampTest")


public class ClampTest extends OpMode {
    ControlledLift controlledLift;
    HardwareClampTest robot;
    boolean lastState = false;

    @Override
    public void init() {
        super.msStuckDetectLoop = 5000000;
        this.controlledLift = new ControlledLift(hardwareMap, telemetry, () -> true);
        this.robot = new HardwareClampTest(hardwareMap);
    }

    @Override
    public void loop() {
        double speed = 0.2;
        double distance = 5;
        int timeout = 20; // not working yet


        if (robot.button.getState() == false) {
            if (lastState) {
                controlledLift.raiseDistance(distance, speed, timeout);
                lastState = false;
            } else {
                controlledLift.raiseDistance(-distance, speed, timeout);
                lastState = true;
            }
            while (!robot.button.getState()) {}
        }
    }
}
