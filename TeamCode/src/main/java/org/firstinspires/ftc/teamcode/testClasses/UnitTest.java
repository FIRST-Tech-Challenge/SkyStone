package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Scanner;

@TeleOp
public class UnitTest extends LinearOpMode {
    Chassis chassis;
    Hook hook;

    @Override
    public void runOpMode() {
        Chassis chassis;
        try {
            chassis = new Chassis(hardwareMap, "/res/config/chassisMotors.txt");
        } catch (IOException e) {
            chassis = new Chassis();
            e.printStackTrace();
        }
        telemetry.setMsTransmissionInterval(1);
        telemetry.addLine("Init | v1.0");
        Iterator<DcMotor> motorIterator= hardwareMap.dcMotor.iterator();
        while (motorIterator.hasNext()){
            DcMotor motor = motorIterator.next();
            telemetry.addData("Motor Name: ",motor.toString());
            telemetry.addLine("Direction: Forward");
            motor.setPower(1);
            sleep(1000);
            telemetry.addLine("Direction: Reverse");
            motor.setPower(-1);
            sleep(1000);
        }
        waitForStart();
        while (opModeIsActive()) {
            //Other unit test code if you want
        }
    }
}