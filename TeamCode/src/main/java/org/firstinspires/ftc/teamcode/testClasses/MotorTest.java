package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autoRes.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;

@TeleOp(name = "Motor Test", group = "Teleop")
public class MotorTest extends LinearOpMode {
    long timeTo = System.currentTimeMillis();

    @Override
    public void runOpMode() {
        Servo intake = hardwareMap.servo.get("wrist");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("wrist:", intake.getPosition());
            //Other unit test code if you want
            telemetry.update();
        }
    }
}