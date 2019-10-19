// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="CoolerCameraTricksOpMode", group="Exercises")

public class CoolerCameraTricksOpMode extends LinearOpMode {

    TensorFlowBot robot = new TensorFlowBot(this);

    @Override
    public void runOpMode() {
        this.robot.init(hardwareMap);

        waitForStart();

        robot.driveStraightByDistance( robot.DIRECTION_RIGHT, 500, 0.3);

        robot.driveUntilSeeSkystone(robot.DIRECTION_FORWARD, -0.08);

        robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 200, 0.3);

        robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 400, 0.3);
    }

}
