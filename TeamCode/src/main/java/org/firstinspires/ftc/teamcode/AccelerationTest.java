package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Acceleration Test", group="Tests")

public class AccelerationTest extends LinearOpMode {
    private FourWheelsDriveBot robot = new FourWheelsDriveBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        robot.driveByDistanceWithAcceleration(robot.DIRECTION_FORWARD, 1000, 0.8, 4);

    }
}