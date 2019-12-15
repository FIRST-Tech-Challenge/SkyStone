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

        robot.drivetrain.autoRotate(90, 0.25);
        sleep(5000);
        robot.drivetrain.autoRotate(180, 0.25);
        sleep(5000);
        robot.drivetrain.autoRotate(270, 0.25);
        sleep(5000);
        robot.drivetrain.autoRotate(0, 0.25);
        sleep(5000);
    }

    @Override
    public void internalPostLoop()
    {
        try
        {
            robot.angleTelemetry();
        }
        catch (NullPointerException ignored) { }
    }
}
