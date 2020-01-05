package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/***
 * To use wireless ADB:
 * ADB CONNECT
 * adb tcpip 5555
 * adb connect 192.168.49.1:5555
 */
@TeleOp(name="FeederTeleOp")
//@Disabled
public class FeederTeleOp extends OpMode {

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
        robot.mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        // runs intake on second driver left stick
        robot.intake(gamepad1.right_trigger - gamepad1.left_trigger);

    }
}
