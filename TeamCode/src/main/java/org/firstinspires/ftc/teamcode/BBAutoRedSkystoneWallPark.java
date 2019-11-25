package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.navigation.Waypoint;

@Autonomous(name="Auto-SkyWall-Red", group="BB")
public class BBAutoRedSkystoneWallPark extends LinearOpMode
{
    private BBSRobot robot = new BBSRobot();


    @Override
    public void runOpMode()
    {

        robot.init(hardwareMap, telemetry, this);

        waitForStart();

        //code below is our auto

        
    }
}
