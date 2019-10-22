package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;

import java.util.EnumMap;

@TeleOp(name="SliderComponentTest", group="TeleOp")
public class SliderComponentTest extends OpMode {

    private ArmSystem armSystem;

    public void init() {
        EnumMap<ArmSystem.ServoNames, Servo> servos = new EnumMap<ArmSystem.ServoNames, Servo>(ArmSystem.ServoNames.class);
        servos.put(ArmSystem.ServoNames.GRIPPER, hardwareMap.get(Servo.class, "gripper"));
        servos.put(ArmSystem.ServoNames.PIVOT, hardwareMap.get(Servo.class, "pivot"));
        servos.put(ArmSystem.ServoNames.WRIST, hardwareMap.get(Servo.class, "wrist"));
        servos.put(ArmSystem.ServoNames.ELBOW, hardwareMap.get(Servo.class, "elbow"));
        armSystem = new ArmSystem(servos, hardwareMap.get(DcMotor.class, "slider_motor"), hardwareMap.get(DigitalChannel.class, "slider_switch"));
    }

    public void loop() {
        if (gamepad1.a) {
            armSystem.setSliderHeight(0);
        } else if (gamepad1.b) {
            armSystem.setSliderHeight(1);
        } else if (gamepad1.x) {
            armSystem.setSliderHeight(2);
        } else if (gamepad1.y) {
            armSystem.setSliderHeight(3);
        }
        armSystem.updateHeight();
    }

}
