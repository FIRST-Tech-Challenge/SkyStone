package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Tests the Pinch Arm and Tensor Flow

@Autonomous(name="Skystone Grab Test", group="Exercises")

public class SkystoneGrabTest extends LinearOpMode {
    protected TensorFlowBot robot = new TensorFlowBot(this);

    int direction_forward, direction_backward;


    protected void setDirection() {
        direction_forward = robot.DIRECTION_FORWARD;
        direction_backward = robot.DIRECTION_BACKWARD;
    }


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        setDirection();
        robot.driveStraightByDistance( robot.DIRECTION_LEFT, 450, 0.7);
        robot.opMode.sleep(1800);
        if (robot.isSkystoneDetected()) {
            robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 170, 0.6);
            robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
            robot.pickupSkyStone();
            robot.opMode.sleep(5000);
            robot.dropSkyStone();
            robot.originalPosition();
        }
        else {
            robot.driveStraightByDistance(direction_forward, 200, 0.6);
            robot.opMode.sleep(1800);
            if (robot.isSkystoneDetected()) {
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 170, 0.6);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.pickupSkyStone();
                robot.opMode.sleep(5000);
                robot.dropSkyStone();
                robot.originalPosition();
            }
            else {
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 300, 0.6);
                robot.driveStraightByDistance(robot.DIRECTION_LEFT, 250, 0.5);
                robot.pickupSkyStone();
                robot.opMode.sleep(5000);
                robot.dropSkyStone();
                robot.originalPosition();
            }
        }

    }

}