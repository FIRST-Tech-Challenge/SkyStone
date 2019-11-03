package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotlib.robot.MecanumRobot;

@Autonomous(name="Mecanum Auto V-Park", group="Auto")
public class MecanumAutoPark extends LinearOpMode
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
        telemetry.addData("Status", "Move to bridge - forward 4 in + left 1 ft");
        robot.robotMove(0, 1, 0, 27);
        robot.robotMove(90, 1, 0, 12);
    }

    // robot move command moved to the robot map
}
