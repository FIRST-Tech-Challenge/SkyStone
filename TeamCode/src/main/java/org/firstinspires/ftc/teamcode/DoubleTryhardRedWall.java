package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class DoubleTryhardRedWall extends LinearOpMode {
    private TensorFlowBot robot = new TensorFlowBot(this);


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

        }

        robot.driveStraightByDistance( robot.DIRECTION_RIGHT, 500, 0.3);
        robot.driveUntilSeeSkystone(-0.08, true);
        robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 200, 0.3);
        robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 400, 0.3);

        robot.pickupSkyStone();

        robot.driveStraightByDistance(robot.DIRECTION_LEFT, 400, 0.5);
        robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 3000, 1);
        robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 200, 0.2);

        robot.dropSkyStone();

        robot.driveStraightByDistance(robot.DIRECTION_LEFT, 200, 1);
        robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 2800, 0.9);
        robot.driveUntilSeeSkystone(-0.08, true);
        robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 200, 0.3);
        robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 400, 0.3);

        robot.pickupSkyStone();

        robot.driveStraightByDistance(robot.DIRECTION_LEFT, 400, 0.5);
        robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 3000, 1);
        robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 200, 0.2);

        robot.dropSkyStone();

        robot.driveStraightByDistance(robot.DIRECTION_LEFT, 200, 1);
        robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 1500, 1);




    }
}
