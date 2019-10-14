package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.RobotMap.*;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Scanner;

@TeleOp
public class IntegrationTest extends LinearOpMode {
    Chassis chassis;
    Hook hook;

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap, new HashMap<ChassisMotor, String>() {{
            put(ChassisMotor.FRONT_LEFT, "front_left_drive");
            put(ChassisMotor.FRONT_RIGHT, "front_right_drive");
            put(ChassisMotor.BACK_LEFT, "back_left_drive");
            put(ChassisMotor.BACK_RIGHT, "back_right_drive");
        }});
        Hook hook = new Hook(hardwareMap, new HashMap<HookServo, String>() {{
            put(HookServo.MAIN, "hook");
        }});
        Intake intake = new Intake(hardwareMap, new HashMap<IntakeMotor, String>() {{
            put(IntakeMotor.LEFT, "left_intake");
            put(IntakeMotor.RIGHT, "right_intake");
        }});
        telemetry.setMsTransmissionInterval(1);
        telemetry.addLine("Init | v1.0");
        Iterator<DcMotor> motorIterator = hardwareMap.dcMotor.iterator();
        while (motorIterator.hasNext()) {
            DcMotor motor = motorIterator.next();
            telemetry.addData("Motor Name: ", motor.toString());
            telemetry.addLine("Direction: Forward");
            motor.setPower(1);
            sleep(100);
            telemetry.addLine("Direction: Reverse");
            motor.setPower(-1);
            sleep(100);
            motor.setPower(0);
        }
        Controller controller = new Controller(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            //Other unit test code if you want
            telemetry.update();
        }
    }
}