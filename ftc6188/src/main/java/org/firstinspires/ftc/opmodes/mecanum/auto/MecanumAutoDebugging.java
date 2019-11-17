package org.firstinspires.ftc.opmodes.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

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

        telemetry.addData("> Status", "60 In Forward");
        robot.drivetrain.autoPosition(0, 60, 1, 0);
    }

    @Override
    public void internalPostLoop()
    {
        telemetry.addData("Loop Status", "IPL");
        robot.informationUpdate();
    }
}
