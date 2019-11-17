package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TryhardRedWall extends LinearOpMode {
    private PinchArmBot robot = new PinchArmBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            robot.driveStraightByDistance( robot.DIRECTION_RIGHT, 500, 0.3);
            robot.driveUntilSeeSkystone(robot.DIRECTION_FORWARD, -0.08);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 200, 0.3);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 400, 0.3);

            robot.pickupSkyStone();

            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 2000, 0.8);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.3);

            robot.dropSkyStone();

            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 1000, 0.6);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 1000, 0.8);
        }
    }
}
