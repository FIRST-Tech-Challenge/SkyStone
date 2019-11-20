package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.MathFunctions;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;

@Autonomous(name="BlueLeft", group ="LinearOpmode")
public class BlueLeft extends AutoBase {
    // transport two skystones and other stones if time permits

    // park in building zone (even if other team is in corner)
    @Override
    public void runOpMode() {
        initLogic();

        robot.moveToPoint(11.5 ,0,1,1,Math.toRadians(0));

        intake(true);

        //int position = robot.runDetection();

        int position = 1;

        double firstSkyStoneY = 0.0;
        double secondSkyStoneY = -24.0;
        if (position == 2){
            firstSkyStoneY = 3.0;
            secondSkyStoneY = -12.0;
            robot.moveToPoint(55,firstSkyStoneY,0.5,0.5,0);
        } else if (position == 1){
            firstSkyStoneY = 0.0;
            secondSkyStoneY = -15.0;
            robot.moveToPoint(55, firstSkyStoneY, 0.5, 0.5,0);
        } else {
            firstSkyStoneY = -3.0;
            secondSkyStoneY = -35.0;
            robot.moveToPoint(55,firstSkyStoneY,0.5,0.5,0);
        }


        double[][] toFoundation = {{55.0,firstSkyStoneY},{10.0,10.0},{15.0,30.0},{15.0,79.0},{29.0,78.0}};
        robot.moveFollowCurveWithExtend(toFoundation, Math.toRadians(-179), 25, Math.toRadians(180), 35, new Point(16.0,60.0));
        intake(false);

        retractOuttake();

        intake(true);
        double [][] toSecondStone = {{5.0,78.0},{10.0,79.0},{28.0,30.0},{25.0,-22.0}};
        robot.moveFollowCurveWithRetract(toSecondStone, Math.toRadians(0),20,Math.toRadians(-90),0,new Point(29.0, 78.0));

        robot.moveToPoint(80.0,secondSkyStoneY,0.5,1,Math.toRadians(-25));

        double[][] toDepositSecondStone = {{40.0,secondSkyStoneY},{15.0,10.0},{15.0,30.0},{15.0,70.0}};
        robot.moveFollowCurveWithExtend(toDepositSecondStone,Math.toRadians(-179),20,Math.toRadians(-90),10,new Point(12.0,50.0));
        intake(false);

        retractOuttake();

        double[][] toParking = {{15.0,70.0},{15.0,40.0}};
        robot.moveFollowCurve(toParking, Math.toRadians(0),20, Math.toRadians(90),0);

    }
}