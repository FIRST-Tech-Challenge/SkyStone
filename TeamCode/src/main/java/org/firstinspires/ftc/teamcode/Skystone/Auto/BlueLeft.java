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
            secondSkyStoneY = -21.0;
            robot.moveToPoint(55,firstSkyStoneY,0.6,0.5,0);
        } else if (position == 1){
            firstSkyStoneY = 0.0;
            secondSkyStoneY = -24.0;
            robot.moveToPoint(55, firstSkyStoneY, 0.6, 0.5, Math.toRadians(0));
        } else {
            firstSkyStoneY = -3.0;
            secondSkyStoneY = -27.0;
            robot.moveToPoint(55,firstSkyStoneY,0.6,0.5,0);
        }
        intake(false);

        double[][] toFoundation = {{55.0,firstSkyStoneY},{6.0,5.0},{5.0,30.0},{30.0,80}};
        PathPoints pathToFoundation = new PathPoints(toFoundation,20);
        robot.moveFollowCurve(pathToFoundation.targetPoints,Math.toRadians(180),Math.toRadians(-180),15);

        intake(true);

        double [][] toSecondStone = {{48.0,80.0},{6.0,40.0},{6.0,30.0},{6.0,5.0},{30.0,secondSkyStoneY}};
        PathPoints pathToSecondStone = new PathPoints(toSecondStone,12);
        robot.moveFollowCurve(pathToSecondStone.targetPoints,Math.toRadians(0),Math.toRadians(-90),15);

        intake(false);

        double[][] toDepositSecondStone = {{48.0,secondSkyStoneY},{6.0,5.0},{6.0,30.0},{15.0,80.0}};
        PathPoints pathToDepositSecondStone = new PathPoints(toDepositSecondStone,20);
        robot.moveFollowCurve(pathToDepositSecondStone.targetPoints, Math.toRadians(180),Math.toRadians(90), 15);

        robot.moveToPoint(15.0, 30.0,1,0,Math.toRadians(180));

//
//        // Determine position of SkyStone
//        int vuforiaPosition = robot.detectTensorflow();
//
//        // Go to the Skystone and intake it
//        goToSkystone(vuforiaPosition,0);
//
//        // Move to the other side of the skybridge
//        robot.moveToPoint(0, 47, 1, 0, Math.toRadians(20));
//
//        telemetry.addLine("return");
//        telemetry.update();
//        telemetry.addLine("DONEEEEE");
//        telemetry.update();
//
//        // Deposit the stone, then retract the outtake
//        depositStone();
//        retractOuttake();
//
//        // Move to the second set of three stones, in anticipation of picking up the second Skystone
//        robot.moveToPoint(0,24,1,1,Math.toRadians(0));
//
//        // Go to the Skystone and intake it
//        goToSkystone(vuforiaPosition,1);
//
//        // Move to the other side of the Skybridge
//        robot.moveToPoint(0, 47, 1, 0, Math.toRadians(20));
//
//
//        // Deposit the Skystone and retract the outtake arm
//        depositStone();
//        retractOuttake();
//
//        // Return for more Stones
//        // TODO: tensorflow for more stones
    }
}