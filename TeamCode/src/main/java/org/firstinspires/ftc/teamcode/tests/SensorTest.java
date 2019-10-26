package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "SensorTest", group = "Test")
public class SensorTest extends OpMode {
    private DigitalChannel limitSwitch;
    public void init() {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "switch");
    }
    public void loop() {
        telemetry.addData("Current Position: ", limitSwitch.getState());
        telemetry.update();
    }
}
