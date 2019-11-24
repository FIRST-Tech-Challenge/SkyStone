package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SensorTest", group = "Test")
public class SensorTest extends OpMode {
    private DigitalChannel limitSwitch;
    private PwmControl servo;
    public void init() {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "switch");
        servo = (PwmControl) hardwareMap.get(Servo.class, "servo");
    }
    public void loop() {
//        PwmControl.PwmRange rangeZero = new PwmControl.PwmRange(0, 100, 0);
        servo.setPwmRange(PwmControl.PwmRange.defaultRange);
        servo.setPwmEnable();
        Log.d("SensorTest", "" + servo.isPwmEnabled());
    }
}
