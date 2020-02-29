package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;

import java.util.ArrayList;

@Autonomous(name = "BackSideBlueFoundationAuto", group = "LinearOpMode")
public class BackSideBlueFoundationAuto extends AutoBase {
    @Override
    public void runOpMode() {
        initLogic();

        waitForStart();

        position2D.startOdometry();
        robot.intake(true);

        while(opModeIsActive() ){

        }
        stop();
        double[][] toFoundation = {
                {0, 0},
                {-15, 8},
                {-32, 10}
        };
        ArrayList<Action> toFoundationActions = new ArrayList<>();
        toFoundationActions.add(new Action(ActionType.EXTEND_FOUNDATION, robot, true));

        double[][] toDepositFoundation = {
                toFoundation[toFoundation.length - 1],
                {-15, 0},
                {-6, -30}
        };
        ArrayList<Action> toDepositFoundationActions = new ArrayList<>();

        robot.splineMove(toFoundation, 1, 1, .4, 20, Math.toRadians(180), Math.toRadians(0), 20, toFoundationActions, true, 1250);

        robot.foundationMovers(true);

        sleep(750);

        robot.splineMove(toDepositFoundation, 1, 1, .7, 20, 0, Math.toRadians(270), 20, toDepositFoundationActions, true, 3000);

        robot.brakeRobot();
        robot.foundationMovers(false);

        sleep(1000);
    }
}
