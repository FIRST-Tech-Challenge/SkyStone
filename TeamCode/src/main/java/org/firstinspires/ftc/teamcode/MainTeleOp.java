package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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

    private Servo servo;
    private double encoder;
    //private Servo chad;

    public void init() {
        robot = new Maccabot(this);
        robot.initializeRobot();
        servo = hardwareMap.servo.get("servo");
        //chad = hardwareMap.servo.get("chad");
        encoder = 0;

    }

    public void loop() {
        // Run a mecanum drive on the first gamepad - left stick translates, right stick rotates
        robot.mecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        // runs intake on second driver left stick
        robot.intake(gamepad2.left_stick_y);
        // runs rack on second driver right stick
        robot.servo(gamepad2.right_stick_y);
        // runs lift on second driver triggers
        robot.lift(gamepad2.left_trigger,gamepad2.right_trigger);
        // runs claw on second driver bumpers
        robot.chad(gamepad2.right_bumper, gamepad2.left_bumper);
    }
}
