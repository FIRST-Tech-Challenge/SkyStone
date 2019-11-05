package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

import java.util.EnumMap;

@TeleOp(name = "Real Teleop", group="TeleOp")
public class DriveTeleop extends BaseOpMode {

    public void init() {
        super.init();
    }


    public void loop(){
        float rx = (float) Math.pow(gamepad1.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad1.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad1.left_stick_y, 3);
        driveSystem.drive(rx, lx, -ly, gamepad1.x);
        spinnySystem.spin(gamepad1.left_bumper, gamepad1.right_bumper);
        latchSystem.run(gamepad2.x, gamepad2.y);
        String armReturn = armSystem.run(gamepad2.b, gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.right_bumper, gamepad2.left_bumper, gamepad2.a,
                true,1, 0.005);
        telemetry.addData("", armReturn);
    }
}