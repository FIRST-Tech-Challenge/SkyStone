package org.firstinspires.ftc.teamcode.Skystone.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import java.util.ArrayList;

@Autonomous(name="StuckTest ", group="LinearOpmode")
public class StuckTest extends LinearOpMode {
    Robot robot;
    Position2D position2D;

    @Override
    public void runOpMode(){
        initLogic();

        waitForStart();

        position2D.startOdometry();

        double[][] toFirstStone = {
                {0,0},
                {48, 0}};
        ArrayList<Action> toFirstStoneActions = new ArrayList<>();

        robot.splineMove(toFirstStone, .9, 1, .9, 0, 0,0,0, toFirstStoneActions, false, 0);
        robot.brakeRobot();

        sleep(500);
    }

    private void initLogic() {
        robot = new Robot(hardwareMap,telemetry,this);
        robot.driveMotorsBreakZeroBehavior();

        position2D = new Position2D(robot);
    }
}