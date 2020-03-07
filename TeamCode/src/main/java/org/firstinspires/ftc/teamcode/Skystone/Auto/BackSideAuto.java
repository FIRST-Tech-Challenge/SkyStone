package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Action;
import org.firstinspires.ftc.teamcode.Skystone.Auto.Actions.Enums.ActionType;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;

import java.util.ArrayList;

@Autonomous(name = "BackSideAuto", group = "LinearOpmode")
public class BackSideAuto extends AutoBase {
    @Override
    public void runOpMode() {
        initLogic();

        double waitTime = 0;

        while (!isStarted()) {
            if (gamepad2.dpad_up) {
                waitTime += 0.05;
            } else if (gamepad2.dpad_down) {
                waitTime -= 0.05;
            }
            telemetry.addLine("WaitTimeInMillis: " + waitTime);
            telemetry.update();
        }

        waitForStart();

        position2D.startOdometry();

        sleep(250);
        sleep((int) Math.abs(waitTime));

        double[][] toPark = {
                {0, 0},
                {12, -.5}};
        ArrayList<Action> toParkActions = new ArrayList<>();

        robot.splineMove(toPark, 0.85, 1, 0.5, 3, 0, 0, 24, toParkActions, true, 5000);

        sleep(500);
    }


}