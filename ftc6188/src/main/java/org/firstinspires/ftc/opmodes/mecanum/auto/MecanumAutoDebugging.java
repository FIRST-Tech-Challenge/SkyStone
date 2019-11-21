package org.firstinspires.ftc.opmodes.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;
import org.firstinspires.ftc.robotlib.state.Button;

@Autonomous(name="Mecanum Auto V-Debug", group="Auto")
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

        telemetry.addData("> Status", "36 in front");
        robot.drivetrain.autoPosition(0, 36, 0);

        telemetry.addData("> Status", "36 in rear");
        robot.drivetrain.autoPosition(180, 36, 0);

        telemetry.addData("> Status", "36 in right");
        robot.drivetrain.autoPosition(270, 36, 0);

        telemetry.addData("> Status", "36 in left");
        robot.drivetrain.autoPosition(90, 36, 0);
    }

    @Override
    public void internalPostLoop()
    {
        telemetry.addData("Loop Status", "IPL");
        robot.informationUpdate();
    }
}
