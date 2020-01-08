package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Maccabot;

/***
 * To use wireless ADB:
 * ADB CONNECT
 * adb tcpip 5555
 * adb connect 192.168.49.1:5555
 */
@TeleOp(name="MainTeleOp")
//@Disabled
public class MainTeleOp extends OpMode {

    Maccabot robot;

    //private Servo moveServo;
    //private double encoder;
    //private Servo moveChad;

    public void init() {
        robot = new Maccabot(this);
        robot.initializeRobot();
        //moveServo = hardwareMap.moveServo.get("moveServo");
        //moveChad = hardwareMap.moveServo.get("moveChad");
        //encoder = 0;

    }

    public void loop() {
        // Run a mecanum drive on the first gamepad - left stick translates, right stick rotates
        robot.mecanumDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
        // runs intake on second driver left stick
        robot.intake(gamepad2.left_stick_y);
        // runs rack on second driver right stick
        robot.moveServo(gamepad2.right_stick_y);
        // runs lift on second driver triggers
        robot.lift(gamepad1.left_trigger,gamepad1.right_trigger);
        // runs claw on second driver bumpers
        robot.moveChad(gamepad2.a, gamepad2.b);
    }
}
