package org.firstinspires.ftc.opmodes.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumFieldGoalRobot;
import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@Autonomous(name="Mecanum Auto V-Comp", group="Auto")
public class MecanumAuto extends LinearOpMode
{
    // Robot
    private MecanumFieldGoalRobot robot;

    // Fields
    private static final double VELOCITY = 0.35;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumFieldGoalRobot(this.hardwareMap, this.telemetry, false);
        waitForStart();

        robot.drivetrain.autoPosition(270, 48, VELOCITY, 0);
        robot.platformServos.setPosition(1);
        robot.drivetrain.autoPosition(90, 24, VELOCITY, 0);
        robot.platformServos.setPosition(0);
        robot.drivetrain.autoPosition(180, 24, VELOCITY, 0);
        robot.drivetrain.autoPosition(270, 20, VELOCITY, 0);
        robot.drivetrain.autoPosition(0, 24, VELOCITY, 0);
        robot.drivetrain.autoPosition(90, 36, VELOCITY, 0);
        robot.drivetrain.autoPosition(180, 36, VELOCITY, 0);
    }
}
