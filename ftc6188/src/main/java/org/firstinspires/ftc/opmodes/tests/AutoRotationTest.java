package org.firstinspires.ftc.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumIMURobot;

@Autonomous(name="Auto Rotation Test", group="Test")
public class AutoRotationTest extends LinearOpMode
{
    // Robot reference
    private SiBorgsMecanumIMURobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new SiBorgsMecanumIMURobot(this.hardwareMap, this.telemetry);

        while (!isStarted()) { robot.angleTelemetry(); }

        robot.drivetrain.setTargetHeading(90);
        sleep(1000);
        robot.drivetrain.setTargetHeading(180);
        sleep(1000);
        robot.drivetrain.setTargetHeading(270);
        sleep(1000);
        robot.drivetrain.setTargetHeading(0);
        sleep(1000);
    }

    @Override
    public void internalPostLoop()
    {
        robot.angleTelemetry();
    }
}
