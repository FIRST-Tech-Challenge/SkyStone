package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RedLeft", group ="Concept")
public class RedRight extends AutoBase{

    @Override
    public void runOpMode() {
        initLogic();

        // Detect position of Skystone
        int vuforiaPosition = robot.detectTensorflow();

        // Go to Skystone and intake it
        goToSkystone(vuforiaPosition,0);

        // Move to other side of the Skybridge
        robot.moveToPoint(7, -47, 1,  1, Math.toRadians(90));

        // Deposit the Skystone and retract the outtake arm
        depositStone(robot);
        retractOuttake(robot);

        // Move back to the second set of Skystones to pick up the second Skystone
        robot.moveToPoint(0,24,1,1,Math.toRadians(0));

        // Go to the Skystone and intake it
        goToSkystone(vuforiaPosition,-1); // Negative one because second set of skystones is to the left of the robot

        // Move to other side of the Skybridge
        robot.moveToPoint(0, -47, 1,  1, Math.toRadians(90));

        // Deposit the Skystone and retract the outtake arm
        depositStone(robot);
        retractOuttake(robot);
    }
}
