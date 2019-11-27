package org.firstinspires.ftc.opmodes.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;

@Autonomous(name="Mecanum Auto V-CompetitionReady", group="AutoComp")
public class SiBorgsMecanumAuto extends LinearOpMode
{
    // Robot
    private SiBorgsMecanumRobot robot;

    // Fields
    private static final double VELOCITY = 0.5;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new SiBorgsMecanumRobot(this.hardwareMap, this.telemetry);
        waitForStart();

        robot.drivetrain.autoPositionWithTelemetry(270, 48, VELOCITY, robot);
        sleep(1000);
        robot.platformServo.setPosition(1);
        sleep(1000);
        robot.drivetrain.autoPositionWithTelemetry(90, 24, VELOCITY, robot);
        sleep(1000);
        robot.platformServo.setPosition(0);
        sleep(1000);
        robot.drivetrain.autoPositionWithTelemetry(180, 24, VELOCITY, robot);
        sleep(1000);
        robot.drivetrain.autoPositionWithTelemetry(270, 20, VELOCITY, robot);
        sleep(1000);
        robot.drivetrain.autoPositionWithTelemetry(0, 24, VELOCITY, robot);
        sleep(1000);
        robot.drivetrain.autoPositionWithTelemetry(90, 36, VELOCITY, robot);
        sleep(1000);
        robot.drivetrain.autoPositionWithTelemetry(180, 36, VELOCITY, robot);
        sleep(1000);

        requestOpModeStop();
    }
}
