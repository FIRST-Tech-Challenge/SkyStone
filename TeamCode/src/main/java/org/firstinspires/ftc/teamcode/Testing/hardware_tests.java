package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassis;
import org.firstinspires.ftc.teamcode.HardwareMaps.HardwareChassisGyro;

@TeleOp (name = "hardware_tests")
public class hardware_tests extends OpMode {
    Servo servo_port0 = null;

    @Override
    public void init() {
        servo_port0 = hardwareMap.get(Servo.class, "servo_port0");
        //TO DO: Neue Hardwaremap für Robot erstellen!!
    }

    @Override
    public void loop(){
        if (gamepad1.a) {
            servo_port0.setPosition(0.6);
        }
        else if (gamepad1.b) {
            servo_port0.setPosition(0.1);
        }
    }

}
