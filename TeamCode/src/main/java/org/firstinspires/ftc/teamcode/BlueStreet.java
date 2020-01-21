package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="New Blue Program", group="New Auto")

public class BlueStreet extends LinearOpMode {

    protected GyroBot robot = new GyroBot(this);

    int direction_forward, direction_backward;

    int[] distFirstMove = new int[]{-100, 100, 300};
    int[] distFoundMove = new int[]{1600, 1800, 2000};
    int[] distBackMove = new int[]{2200, 2400, 1600};

    int skystonePostition;
    protected void setDirection(){
        direction_backward = robot.DIRECTION_FORWARD;
        direction_forward = robot.DIRECTION_BACKWARD;

    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        setDirection();
        skystonePostition = robot.detectSkystone();
            robot.driveUntilDistance(18, 0.3, 1);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, distFirstMove[skystonePostition - 1]);
            robot.pickupSkyStone();
//            robot.driveUntilDistance(35, 0.3, 1);

//            robot.driveStraightByDistance(direction_forward, distFoundMove[skystonePostition - 1], 0.8);

            robot.driveByDistanceWithAcceleration(direction_forward, distFoundMove[skystonePostition - 1], 1, 10);

            robot.goBacktoStartAngle();

            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 200, 0.5);
            robot.dropSkyStone();
            robot.originalPosition();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 200, 0.5);
//            robot.driveStraightByDistance(direction_backward, distBackMove[skystonePostition - 1], 0.8);

            robot.driveByDistanceWithAcceleration(direction_backward, distBackMove[skystonePostition - 1], 1, 10);

            robot.goBacktoStartAngle();

            sleep(1 * 1000);

            robot.driveUntilDistance(18, 0.3, 1);

            robot.pickupSkyStone();

//            robot.driveUntilDistance(35, 0.3, 1);
//            robot.driveStraightByDistance(direction_forward, distBackMove[skystonePostition - 1], 0.8);

            robot.driveByDistanceWithAcceleration(direction_forward, distBackMove[skystonePostition - 1], 1, 10);

            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 200, 0.5);
            robot.dropSkyStone();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 200, 0.5);
            robot.driveStraightByDistance(direction_backward, 800, 1);

        }


    }

