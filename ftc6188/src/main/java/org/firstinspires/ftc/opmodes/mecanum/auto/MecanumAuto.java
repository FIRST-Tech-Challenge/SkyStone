package org.firstinspires.ftc.opmodes.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumFieldGoalRobot;

@Autonomous(name="Mecanum Auto V-Comp", group="Auto")
public class MecanumAuto extends LinearOpMode
{
    private MecanumFieldGoalRobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumFieldGoalRobot(this.hardwareMap, this.telemetry, false);
        waitForStart();

        robot.drivetrain.autoPosition(0, 48, 1, 0);
        robot.platformServos.setPosition(1);
        robot.drivetrain.autoPosition(180, 24, 1, 0);
        robot.platformServos.setPosition(0);
        robot.drivetrain.autoPosition(90, 24, 1, 0);
        robot.drivetrain.autoPosition(0, 20, 1, 0);
        robot.drivetrain.autoPosition(270, 24, 1, 0);
        robot.drivetrain.autoPosition(180, 36, 1, 0);
        robot.drivetrain.autoPosition(90, 36, 1, 0);
    }
}
