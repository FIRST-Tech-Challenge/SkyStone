package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Double SkyStone Blue Wall", group="Exercises")

public class DoubleSkyStoneBlueWall extends LinearOpMode {
    protected TensorFlowBot robot = new TensorFlowBot(this);

    int direction_forward, direction_backward;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        direction_forward = robot.DIRECTION_FORWARD;
        direction_backward = robot.DIRECTION_BACKWARD;

        robot.driveStraightByDistance( robot.DIRECTION_LEFT, 400, 0.7);
        robot.opMode.sleep(1800);
        if (robot.isSkystoneDetected()) {
            robot.driveStraightByDistance(direction_forward, 150, 0.6);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
            robot.pickupSkyStone();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
            robot.driveStraightByDistance(direction_backward, 1600, 0.9);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
            robot.dropSkyStone();
            robot.originalPosition();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 250, 0.8);
            robot.driveStraightByDistance(direction_forward, 2360, 1);
            robot.driveStraightByDistance(direction_forward, 200, 0.3);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 700, 0.5);
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 250, 0.7);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 630, 0.7);
            robot.pickupSkyStone();
            robot.opMode.sleep(500);
            robot.driveStraightByDistance(direction_backward, 1600, 1);
            robot.dropSkyStone();

            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 300, 1);
        } else {
            robot.driveStraightByDistance(direction_forward, 275, 0.6);
            robot.opMode.sleep(1800);
            if (robot.isSkystoneDetected()) {
                robot.driveStraightByDistance(direction_forward, 150, 0.6);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
                robot.pickupSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
                robot.driveStraightByDistance(direction_backward, 1800, 0.8);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.dropSkyStone();
                robot.originalPosition();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 250, 0.9);
                robot.driveStraightByDistance(direction_forward, 2360, 1);
                robot.driveStraightByDistance(direction_forward, 200, 0.3);
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 700, 0.5);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 50, 0.7);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 630, 0.7);
                robot.pickupSkyStone();
                robot.opMode.sleep(500);
                robot.driveStraightByDistance(direction_backward, 1800, 1);
                robot.dropSkyStone();

                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 300, 1);
            } else {
                robot.driveStraightByDistance(direction_forward, 275, 0.6);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
                robot.pickupSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
                robot.driveStraightByDistance(direction_backward, 2000, 0.8);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.dropSkyStone();
                robot.originalPosition();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 250, 0.9);
                robot.driveStraightByDistance(direction_forward, 2360, 1);
                robot.driveStraightByDistance(direction_forward, 200, 0.3);
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 700, 0.5);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 50, 0.7);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 630, 0.7);
                robot.pickupSkyStone();
                robot.opMode.sleep(500);
                robot.driveStraightByDistance(direction_backward, 1800, 1);
                robot.dropSkyStone();

                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 300, 1);
            }
        }


    }
}