package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@Autonomous(name="Mecanum Auto V-Full", group="Auto")
public class MecanumAutoPark extends LinearOpMode
{
    private MecanumRobot robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new MecanumRobot(this.hardwareMap, this.telemetry, false);
        robot.platformServos.setPosition(1); //init servos to up position
        robot.informationUpdate();

        waitForStart();

        // Move commands
        telemetry.addData("Status", "Move to bridge - left 1 ft");
        robot.robotMove(0, 1, 0, 4);
        robot.robotMove(270, 1, 0, 12);
    }

    // robot move command moved to the robot map
}
