package org.firstinspires.ftc.teamcode.TestModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareClampTest;
import org.firstinspires.ftc.teamcode.Library.Movement.ControlledArm;

@TeleOp(name = "ClampTest")


public class ClampTest extends OpMode {
    ControlledArm controlledArm;
    HardwareClampTest robot;
    boolean lastState = false;

    @Override
    public void init() {
        super.msStuckDetectLoop = 5000000;
        this.controlledArm = new ControlledArm(hardwareMap, telemetry);
        this.robot = new HardwareClampTest(hardwareMap);
    }

    @Override
    public void loop() {
        double speed = 0.2;
        double distance = 5;
        int timeout = 20; // not working yet


        if (robot.button.getState() == false) {
            if (lastState) {
                controlledArm.raiseDistance(distance, speed, timeout);
                lastState = false;
            } else {
                controlledArm.raiseDistance(-distance, speed, timeout);
                lastState = true;
            }
            while (!robot.button.getState()) {}
        }
    }
}
