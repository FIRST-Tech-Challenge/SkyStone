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
            robot.driveUntilSeeSkystone(-0.08, true);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 200, 0.3);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 400, 0.3);

            robot.pickupSkyStone();

            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 3000, 1);
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.2);

            robot.dropSkyStone();

            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 100, 1);
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 1500, 0.9); // Drive approximately 1/2 of where u came from (1000/2) values can change (park under bridge)
        }
}
