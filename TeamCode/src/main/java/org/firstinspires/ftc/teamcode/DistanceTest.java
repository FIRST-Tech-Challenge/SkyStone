package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DistanceSensor Test", group="Exercises")

public class DistanceTest extends LinearOpMode {
    private TensorFlowBot robot = new TensorFlowBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.getDistanceFront();

        }
    }
}