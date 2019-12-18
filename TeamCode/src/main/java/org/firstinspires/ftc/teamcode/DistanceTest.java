package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DistanceSensoe", group="Exercises")

public class DistanceTest extends LinearOpMode {
    private DistanceSensorBot robot = new DistanceSensorBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {


        }
    }
}