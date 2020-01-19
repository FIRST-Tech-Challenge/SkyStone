package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous(name="ONLYRUNTHISONE", group="Exercises")

public class RedStreet extends LinearOpMode {

    protected CameraBot robot = new CameraBot(this);

    int direction_forward, direction_backward;

    int[] distFirstMove = new int[]{100, 200, 300};
    int[] distFoundMove = new int[]{1000, 2000, 3000};

    int skystonePostition;
    protected void setDirection(){
        direction_forward = robot.DIRECTION_BACKWARD;
        direction_backward = robot.DIRECTION_FORWARD;

    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        setDirection();
        skystonePostition = robot.detectSkystone();
            robot.driveUntilDistance(21, 0.3);
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, distFirstMove[skystonePostition - 1]);
            robot.pickupSkyStone();
            robot.driveUntilDistance(35, 0.3);

            robot.driveStraightByDistance(direction_forward, 1600, 0.8);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
            robot.dropSkyStone();
            robot.originalPosition();
            robot.driveStraightByDistance(robot.DIRECTION_RIGHT, 150, 0.5);
            robot.driveStraightByDistance(direction_forward, 850, 1);

        }


    }
}
