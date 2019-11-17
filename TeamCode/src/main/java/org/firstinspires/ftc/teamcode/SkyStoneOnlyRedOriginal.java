package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SkyStoneOnlyRedOriginal extends LinearOpMode {
    private PinchArmBot robot = new PinchArmBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

        }
            robot.driveStraightByDistance( robot.DIRECTION_RIGHT, 500, 0.3);
            robot.driveUntilSeeSkystone(robot.DIRECTION_FORWARD, -0.08);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 200, 0.3);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 400, 0.3);

            robot.pickupSkyStone();

            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 10000, 1);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.2);

            robot.dropSkyStone();

            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 100, 1);
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 500, 0.9);
        }
}
