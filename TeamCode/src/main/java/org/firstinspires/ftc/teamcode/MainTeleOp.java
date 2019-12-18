package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Maccabot;

@TeleOp(name="MainTeleOp")
//@Disabled
//ADB CONNECT
//adb tcpip 5555
//adb connect 192.168.49.1:5555
public class MainTeleOp extends OpMode {

    Maccabot robot;

    private Servo servo;
    private double encoder;

    public void init() {
        robot = new Maccabot(this);
        robot.initializeRobot();
        servo = hardwareMap.servo.get("servo");
        encoder = 0;
    }

    public void loop() {
        robot.mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        robot.intake(gamepad2.left_stick_y);
        robot.servo(gamepad2.right_stick_y);

        robot.lift(gamepad2.left_trigger,gamepad2.right_trigger);
    }
}
