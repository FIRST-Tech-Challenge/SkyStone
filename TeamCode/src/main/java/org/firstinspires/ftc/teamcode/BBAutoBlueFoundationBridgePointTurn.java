package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigation.Waypoint;

@Autonomous(name="Auto-Foundation-Blue-Point-Bridge", group="BB")
public class BBAutoBlueFoundationBridgePointTurn extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private BBSRobot robot = new BBSRobot();
    private BBHooks hooks = new BBHooks();

    @Override
    public void runOpMode() {


        robot.init(hardwareMap, telemetry, this);
        hooks.init(hardwareMap);
        hooks.UnLatched();

        robot.RobotMoveY(new Waypoint(0, 72, 0 ), 0.3);
        robot.Stop();

        robot.RobotMoveX(new Waypoint(20, 0, 0 ), 0.3);
        robot.Stop();

        hooks.UnLatched();
        sleep( 1000);
        robot.RightPointTurn(90, 0.2);
        robot.Stop();
    }


}
