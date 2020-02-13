package org.firstinspires.ftc.teamcode.Skystone.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.AutoBase;

import java.util.ArrayList;

@Autonomous(name = "autoNavigationTest", group = "LinearOpMode")
public class autoNavigationTest extends AutoBase {
    @Override
    public void runOpMode() {
        initLogic();

        double[][] movement = {
                {0,0,10,0},
                {10,0,10,0},
                {20,0,10,0}
        };
        ArrayList<Action> movementActions = new ArrayList<>();

        waitForStart();

//        robot.splineMoveTest(movement,1,1,1,0,0,0,0, movementActions, false, 1000);
    }
}
