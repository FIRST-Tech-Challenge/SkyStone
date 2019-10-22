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
    DcMotor motor;
    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.DcMotor.get("arm");


        telemetry.setMsTransmissionInterval(1);
        telemetry.addLine("Init | v1.0");
        waitForStart();
        while (opModeIsActive()) {
            //Other unit test code if you want
            telemetry.update();
        }
    }
}