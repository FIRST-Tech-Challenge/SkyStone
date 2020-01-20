package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Acceleration Backward", group="Tests")

public class AccelerationTest2 extends LinearOpMode {
    private FourWheelsDriveBot robot = new FourWheelsDriveBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        robot.driveByDistanceWithAcceleration(robot.DIRECTION_BACKWARD, 1500, 0.5, 5);

    }
}