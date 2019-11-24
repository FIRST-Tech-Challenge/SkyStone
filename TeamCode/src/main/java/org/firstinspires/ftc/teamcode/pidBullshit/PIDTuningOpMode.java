package org.firstinspires.ftc.teamcode.pidBullshit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Maccabot;

@TeleOp(name="MainTeleOp")
//@Disabled
public class PIDTuningOpMode extends OpMode {

    Maccabot robot;

    private Servo servo;
    private double encoder;

    @Override
    public void init() {
        robot = new Maccabot(this);
        robot.initializeRobot();
        servo = hardwareMap.servo.get("servo");
        encoder = 0;
    }

    @Override
    public void loop() {
        robot.mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        robot.intake(gamepad2.left_trigger, gamepad2.right_trigger);
        robot.servo(0.35, gamepad2.a, gamepad2.b);
        telemetry.addData("servo position",servo.getPosition());
        telemetry.update();
    }
}
