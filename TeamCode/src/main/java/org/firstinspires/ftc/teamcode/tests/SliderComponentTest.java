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
        armSystem = new ArmSystem(
                hardwareMap.get(DcMotor.class, "slider_motor"),
                hardwareMap.get(DigitalChannel.class, "slider_switch"));
    }

    public void init_loop() {
        if (!armSystem.isCalibrated()) {
            armSystem.calibrate();
        }
    }

    public void loop() {
        telemetry.addData("", armSystem.getSwitchState());

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
