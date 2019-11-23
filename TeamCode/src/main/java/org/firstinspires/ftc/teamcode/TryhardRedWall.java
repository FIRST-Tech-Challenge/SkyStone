package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Tryhard Red Wall", group="Exercises")

public class TryhardRedWall extends LinearOpMode {
    private TensorFlowBot robot = new TensorFlowBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();


        robot.driveStraightByDistance( robot.DIRECTION_LEFT, 400, 0.7);
//        robot.driveUntilSeeSkystone(-0.08, false);
        robot.opMode.sleep(2000);
        if (robot.isSkystoneDetected()) {
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 150, 0.6);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
            robot.pickupSkyStone();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 1600, 0.8);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
            robot.dropSkyStone();
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 180, 0.6);
            robot.dragFoundation();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 2000, 0.9);
            robot.resetArm();
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 1000, 1);
        } else {
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 200, 0.6);
            robot.opMode.sleep(2000);
            if (robot.isSkystoneDetected()) {
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 150, 0.6);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
                robot.pickupSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 1800, 0.8);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.dropSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 180, 0.6);
                robot.dragFoundation();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 2000, 0.9);
                robot.resetArm();
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 1000, 1);
            } else {
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 350, 0.6);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 270, 0.5);
                robot.pickupSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.5);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 2000, 0.8);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.dropSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 180, 0.6);
                robot.dragFoundation();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 2000, 0.9);
                robot.resetArm();
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 1000, 1);

            }
        }


    }
}
