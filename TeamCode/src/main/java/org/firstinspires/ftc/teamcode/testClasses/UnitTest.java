package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.Scanner;

@TeleOp
public class UnitTest extends LinearOpMode {
    Chassis chassis;
    Hook hook;

    @Override
    public void runOpMode() {
        //chassis = new Chassis();
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}