package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="One SkyStone Red Wall", group="Exercises")

public class OneSkyStoneRedWall extends LinearOpMode {
    protected TensorFlowBot robot = new TensorFlowBot(this);

    int direction_forward, direction_backward;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        direction_forward = robot.DIRECTION_FORWARD;
        direction_backward = robot.DIRECTION_BACKWARD;

        robot.driveStraightByDistance( robot.DIRECTION_LEFT, 400, 0.7);
        robot.opMode.sleep(2000);
        if (robot.isSkystoneDetected()) {
            robot.driveStraightByDistance(direction_forward, 150, 0.6);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
            robot.pickupSkyStone();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
            robot.driveStraightByDistance(direction_backward, 1600, 0.8);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
            robot.dropSkyStone();
            robot.originalPosition();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 800, 0.9);
            robot.driveStraightByDistance(direction_forward, 850, 1);
        } else {
            robot.driveStraightByDistance(direction_forward, 200, 0.6);
            robot.opMode.sleep(2000);
            if (robot.isSkystoneDetected()) {
                robot.driveStraightByDistance(direction_forward, 150, 0.6);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
                robot.pickupSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
                robot.driveStraightByDistance(direction_backward, 1800, 0.8);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.dropSkyStone();
                robot.originalPosition();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 1200, 0.9);
                robot.driveStraightByDistance(direction_forward, 850, 1);
            } else {
                robot.driveStraightByDistance(direction_forward, 350, 0.6);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
                robot.pickupSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
                robot.driveStraightByDistance(direction_backward, 2000, 0.8);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.dropSkyStone();
                robot.originalPosition();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 1200, 0.9);
                robot.driveStraightByDistance(direction_forward, 850, 1);
            }
        }


    }
}
