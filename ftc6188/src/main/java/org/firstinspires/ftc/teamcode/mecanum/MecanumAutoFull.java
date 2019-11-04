package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@Disabled
@Autonomous(name="Mecanum Auto V-Full", group="Auto")
public class MecanumAutoFull extends LinearOpMode
{
    private MecanumRobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumRobot(this.hardwareMap, this.telemetry, false);
        robot.platformServos.setPosition(0); //init servos to up position
        robot.informationUpdate();

        waitForStart();

        // Move commands
        //TODO: fill out move commands for a full possible auto

        robot.autoPosition(0, 1, 0, 29.5);
        robot.platformServos.setPosition(1);
        robot.autoPosition(180, 1, 0, 29);
        robot.platformServos.setPosition(0);

    }
}
