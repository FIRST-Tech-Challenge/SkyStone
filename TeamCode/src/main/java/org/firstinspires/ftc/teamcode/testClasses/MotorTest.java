package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autoRes.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;

@Autonomous
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        DriveCommand driveCommand = new DriveCommand(chassis,10,15,90,1,150);
        waitForStart();
        while (opModeIsActive()) {
            while(driveCommand.runCommand());
            telemetry.update();
        }
    }
}