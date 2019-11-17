package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Tryhard Red Wall", group="Exercises")

public class TryhardRedWall extends LinearOpMode {
    private PinchArmBot robot = new PinchArmBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();


        robot.driveStraightByDistance( robot.DIRECTION_LEFT, 400, 0.3);
//        robot.driveUntilSeeSkystone(-0.08, false);
        robot.opMode.sleep(2000);
        if (robot.isSkystoneDetected()) {
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 150, 0.3);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.3);
            robot.pickupSkyStone();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.3);
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 1800, 0.5);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 300, 0.3);
            robot.dropSkyStone();
            robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 200, 0.4);
            robot.dragFoundation();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 800, 0.4);
            robot.resetArm();
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 1200, 0.8);
        } else {
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 200, 0.3);
            robot.opMode.sleep(2000);
            if (robot.isSkystoneDetected()) {
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 150, 0.3);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.3);
                robot.pickupSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.3);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 2000, 0.5);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 300, 0.3);
                robot.dropSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 200, 0.4);
                robot.dragFoundation();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 800, 0.4);
                robot.resetArm();
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 1200, 0.8);
            } else {
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 200, 0.3);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 150, 0.3);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.3);
                robot.pickupSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 100, 0.3);
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 2200, 0.5);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 300, 0.3);
                robot.dropSkyStone();
                robot.driveStraightByDistance(robot.DIRECTION_BACKWARD, 200, 0.4);
                robot.dragFoundation();
                robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 800, 0.4);
                robot.resetArm();
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 1200, 0.8);

            }
        }


    }
}
