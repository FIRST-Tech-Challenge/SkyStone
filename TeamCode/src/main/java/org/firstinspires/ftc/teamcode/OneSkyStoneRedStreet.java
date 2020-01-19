package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ONLYRUNTHISONE", group="Exercises")

public class OneSkyStoneRedStreet extends LinearOpMode {
    protected TensorFlowBot robot = new TensorFlowBot(this);

    int direction_forward, direction_backward;

    protected void setDirection(){
        direction_forward = robot.DIRECTION_BACKWARD;
        direction_backward = robot.DIRECTION_FORWARD;
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        setDirection();
//        robot.driveStraightByDistance( robot.DIRECTION_LEFT, 400, 0.7);
        robot.driveUntilDistance(70, 0.3);
        robot.opMode.sleep(3000);
        if (robot.isSkystoneDetected()) {
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 150, 0.6);
//            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
            robot.driveUntilDistance(21, 0.3);
            
            robot.pickupSkyStone();
//            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
            robot.driveUntilDistance(35, 0.3);

            robot.driveStraightByDistance(direction_backward, 1600, 0.8);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
            robot.dropSkyStone();
            robot.originalPosition();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 150, 0.5);
            robot.driveStraightByDistance(direction_forward, 850, 1);
        } else {
            robot.driveStraightByDistance(direction_forward, 200, 0.6);
            robot.opMode.sleep(3000);
            if (robot.isSkystoneDetected()) {
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 150, 0.6);
//                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
                robot.driveUntilDistance(21, 0.3);

                robot.pickupSkyStone();
//                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
                robot.driveUntilDistance(35, 0.3);

                robot.driveStraightByDistance(direction_backward, 1800, 0.8);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.dropSkyStone();
                robot.originalPosition();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 150, 0.5);
                robot.driveStraightByDistance(direction_forward, 850, 1);
            } else {

                robot.driveStraightByDistance(direction_forward, 200, 0.6);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 150, 0.6);
//                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
                robot.driveUntilDistance(21, 0.3);

                robot.pickupSkyStone();
//                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
                robot.driveUntilDistance(35, 0.3);

                robot.driveStraightByDistance(direction_backward, 2000, 0.8);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.dropSkyStone();
                robot.originalPosition();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 150, 0.5);
                robot.driveStraightByDistance(direction_forward, 850, 1);
            }
        }


    }
}
