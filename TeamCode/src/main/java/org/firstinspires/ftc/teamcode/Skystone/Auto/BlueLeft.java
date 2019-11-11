package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;

@Autonomous(name="BlueLeft", group ="LinearOpmode")
public class BlueLeft extends AutoBase {
    // transport two skystones and other stones if time permits

    // park in building zone (even if other team is in corner)
    @Override
    public void runOpMode() {
        initLogic();

        robot.moveToPoint(11.5 ,0,1,1,Math.toRadians(0));

        intake(true);

        //int position = robot.detectTensorflow();

        int position = 1;

        double firstSkyStoneY = 0.0;
        double secondSkyStoneY = -24.0;
        if (position == 2){
            firstSkyStoneY = 3.0;
            secondSkyStoneY = -12.0;
            robot.moveToPoint(55,firstSkyStoneY,0.8,0.5,0);
        } else if (position == 1){
            firstSkyStoneY = 0.0;
            secondSkyStoneY = -15.0;
            robot.moveToPoint(55, firstSkyStoneY, 0.8, 0.5,0);
        } else {
            firstSkyStoneY = -3.0;
            secondSkyStoneY = -18.0;
            robot.moveToPoint(55,firstSkyStoneY,0.8,0.5,0);
        }
        intake(false);

        double[][] toFoundation = {{55.0,firstSkyStoneY},{10.0,10.0},{15.0,30.0},{30.0,80}};
        PathPoints pathToFoundation = new PathPoints(toFoundation,20);
        robot.moveFollowCurve(pathToFoundation.targetPoints,Math.toRadians(-179),Math.toRadians(-60),40);

        intake(true);
        double [][] toSecondStone = {{48.0,80.0},{20.0,30.0},{30.0,5.0},{40.0,secondSkyStoneY}};
        PathPoints pathToSecondStone = new PathPoints(toSecondStone,20);
        robot.moveFollowCurve(pathToSecondStone.targetPoints,Math.toRadians(0));
        intake(false);

        double[][] toDepositSecondStone = {{40.0,secondSkyStoneY},{10.0,10.0},{12.0,30.0},{5.0,80.0}};
        PathPoints pathToDepositSecondStone = new PathPoints(toDepositSecondStone,20);
        robot.moveFollowCurve(pathToDepositSecondStone.targetPoints, Math.toRadians(-179));

        robot.moveToPoint(15.0, 30.0,1,1,Math.toRadians(0));

    }
}