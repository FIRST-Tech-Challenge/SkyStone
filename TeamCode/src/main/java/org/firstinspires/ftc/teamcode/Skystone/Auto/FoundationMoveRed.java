package org.firstinspires.ftc.teamcode.Skystone.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "FoundationMoveRed2", group = "LinearOpMode")
public class FoundationMoveRed extends AutoBase {
    @Override
    public void runOpMode() {
        initLogic();

        waitForStart();

        position2D.startOdometry();

        double[][] toFoundation = {
                {0, 0},
                {-15, -15},
                {-32, -20},
                {-34,-33}
        };
        ArrayList<Action> toFoundationActions = new ArrayList<>();
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, new Point(-34,-33),robot));

        double[][] toDepositFoundation = {
                toFoundation[toFoundation.length - 1],
                {-15, -5},
                {-1, 10}
        };
        ArrayList<Action> toDepositFoundationActions = new ArrayList<>();
        toDepositFoundationActions.add(new Action(ActionType.RELEASE_FOUNDATION, new Point(0,5),robot));

        double[][] park = {
                toDepositFoundation[toDepositFoundation.length - 1],
                {0, 15},
                {0, 35}
        };
        ArrayList<Action> toParkActions = new ArrayList<>();

        robot.splineMove(toFoundation, 1, 0, 0.75, 15, Math.toRadians(180), Math.toRadians(0), 2, toFoundationActions, true, 2000);

        robot.brakeRobot();

        robot.foundationMovers(true);

        sleep(200);

        robot.splineMove(toDepositFoundation, 1, 1, 1, 20, 0, Math.toRadians(90), 20, toDepositFoundationActions, true, 2000);

        robot.foundationMovers(false);

        sleep(2000);

        robot.splineMove(park, 0.7, 1, .4, 25, 0, Math.toRadians(90), 20, toParkActions, true, 10000);

    }
}

