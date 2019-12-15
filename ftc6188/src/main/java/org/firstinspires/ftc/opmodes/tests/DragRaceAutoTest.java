package org.firstinspires.ftc.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumIMURobot;
import org.firstinspires.ftc.robotlib.state.AutoDirection;

@Autonomous(name="Drag Race Auto Test", group="Test")
public class DragRaceAutoTest extends LinearOpMode
{
    private SiBorgsMecanumIMURobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new SiBorgsMecanumIMURobot(this.hardwareMap, this.telemetry);

        waitForStart();

        robot.sirenSound.toggleSound();
        robot.drivetrain.autoPosition(AutoDirection.FRONT, 47*12, 1, 0);
        robot.drivetrain.autoRotate(180, 0.25);
        robot.drivetrain.autoPosition(AutoDirection.FRONT, 47*12, 1, 0);
        robot.drivetrain.autoRotate(180, 0.25);
    }
}
