// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Wall Park Left", group="Exercises")

public class WallParkLeftOpMode extends LinearOpMode {

    FourWheelsDriveBot robot = new FourWheelsDriveBot(this);

    @Override
    public void runOpMode() {
        this.robot.init(hardwareMap);

        waitForStart();
        sleep(1000);
       // robot.driveStraightByDistance( robot.DIRECTION_FORWARD, 200);
       // robot.driveStraightByDistance( robot.DIRECTION_LEFT, 200);
       // robot.driveStraightByDistance( robot.DIRECTION_BACKWARD, 200);
        robot.driveStraightByDistance( robot.DIRECTION_RIGHT, 200);
    }

}
