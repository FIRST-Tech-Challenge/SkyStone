package org.firstinspires.ftc.opmodes.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@Autonomous(name="Mecanum Auto V-Debug", group="Auto")
public class MecanumAutoDebugging extends LinearOpMode
{
    private MecanumRobot robot;
    private static final int STOP_TIME = 5000;
    private static final double VELOCITY = 0.6;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumRobot(this.hardwareMap, this.telemetry, false);
        robot.informationUpdate();
        telemetry.addData("> Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("> Status", "36 in front");
        robot.drivetrain.autoPosition(0, 36, VELOCITY,0);
        sleep(STOP_TIME);
        telemetry.addData("> Status", "36 in left");
        robot.drivetrain.autoPosition(90, 36, VELOCITY,0);
        sleep(STOP_TIME);
        telemetry.addData("> Status", "36 in reverse");
        robot.drivetrain.autoPosition(180, 36, VELOCITY,0);
        sleep(STOP_TIME);
        telemetry.addData("> Status", "36 in right");
        robot.drivetrain.autoPosition(270, 36, VELOCITY,0);
        sleep(STOP_TIME);

        requestOpModeStop();
    }

    @Override
    public void internalPostLoop()
    {
        robot.informationUpdate();
        telemetry.update();
    }
}
