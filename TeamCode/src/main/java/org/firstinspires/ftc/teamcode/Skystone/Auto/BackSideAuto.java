package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;

import java.util.ArrayList;

@Autonomous(name="BackSideAuto", group ="LinearOpmode")
public class BackSideAuto extends AutoBase{
    @Override
    public void runOpMode() {
        initLogic();

        double waitTime = 15000;

        while (!isStarted()){
            if(gamepad2.dpad_up){
                waitTime += 0.05;
            }else if(gamepad2.dpad_down){
                waitTime -=0.05;
            }
            telemetry.addLine("WaitTimeInMillis: " + waitTime);
            telemetry.update();
        }

        waitForStart();

        position2D.startOdometry();

        sleep(250);
        sleep((int)waitTime);

        double[][] toPark = {
                {0,0,10,0},
                {10,0,10,0}};
        ArrayList<Action> toParkActions = new ArrayList<Action>();
        toParkActions.add(new Action(ActionType.EXTEND_OUTTAKE, new Point(0,0), robot));


        robot.splineMove(toPark,1,1,0.5,15,0,Math.toRadians(0),10,toParkActions);

    }


}