package org.firstinspires.ftc.teamcode.mecanum.autocode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@Autonomous(name="Mecanum Auto Debugging", group="Debug")
public class MecanumAutoDebugging extends LinearOpMode
{
    private MecanumRobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumRobot(this.hardwareMap, this.telemetry, false);

        robot.informationUpdate();
        telemetry.addData("> Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("> Status", "12 In Forward");
        robot.drivetrain.autoPosition(0, 36, 1, 0);
        telemetry.addData("> Status", "12 in Reverse");
        robot.drivetrain.autoPosition(180, 36, 1, 0);
    }

    @Override
    public void internalPostLoop()
    {
        telemetry.addData("Loop Status", "IPL");
        robot.informationUpdate();
    }
}
