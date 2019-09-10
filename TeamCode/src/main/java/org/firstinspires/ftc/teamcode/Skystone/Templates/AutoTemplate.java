package org.firstinspires.ftc.teamcode.Skystone.Templates;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Skystone.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;

@Autonomous(name="AutoTemplate ", group="Linear Opmode")
public class AutoTemplate extends AutoBase
{
    private static double[][] testPoints = {{0,-20},{-30,-65}};
    private static double[][] testPoints2 = {{0,-20},{-30,-65},{0,-35}};

    @Override
    public void runOpMode(){
        initLogic();

        PathPoints testPath = new PathPoints(testPoints);

        while (opModeIsActive()){
            robot.moveFollowCurve(testPath.targetPoints);
            //does a new path below
//            robot.moveFollowCurve(testPath.newPoints(testPoints2));
            break;
        }
    }
}

