package org.firstinspires.ftc.opmodes.mecanum.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.SiBorgsMecanumRobot;
import org.firstinspires.ftc.robotlib.state.ServoState;

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

        robot.drivetrain.autoPosition(270, 48, VELOCITY, 0);
        robot.platformServo.setPosition(ServoState.DOWN);
        robot.drivetrain.autoPosition(90, 24, VELOCITY, 0);
        robot.platformServo.setPosition(ServoState.UP);
        robot.drivetrain.autoPosition(180, 24, VELOCITY, 0);
        robot.drivetrain.autoPosition(270, 20, VELOCITY, 0);
        robot.drivetrain.autoPosition(0, 24, VELOCITY, 0);
        robot.drivetrain.autoPosition(90, 36, VELOCITY, 0);
        robot.drivetrain.autoPosition(180, 36, VELOCITY, 0);

        requestOpModeStop();
    }

    @Override
    public void internalPostLoop()
    {
        robot.informationTelemetry("Internal Post Loop Running");
    }
}
