// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="All-In-One Parking", group="Parking")

public class AllInOneParking extends LinearOpMode {

    FourWheelsDriveBot robot = new FourWheelsDriveBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        int position = 0;
        final int STREET_LEFT = 1;
        final int STREET_RIGHT = 2;
        final int WALL_LEFT = 3;
        final int WALL_RIGHT = 4;
        while(! isStarted()) {
            if (gamepad1.dpad_up && gamepad1.x) {
                position = STREET_LEFT;
                telemetry.addData("Position", "STREET LEFT!");
            }
            if (gamepad1.dpad_up && gamepad1.b) {
                position = STREET_RIGHT;
                telemetry.addData("Position", "STREET RIGHT!");
            }
            if (gamepad1.dpad_left) {
                position = WALL_LEFT;
                telemetry.addData("Position", "WALL LEFT!");
            }
            if (gamepad1.dpad_right) {
                position = WALL_RIGHT;
                telemetry.addData("Position", "WALL RIGHT!");
            }
            telemetry.update();
        }
        sleep(1000);

        switch (position) {
            case STREET_LEFT:
                robot.driveStraightByDistance( robot.DIRECTION_LEFT, 200, 0.5);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 300, 0.5);
                break;
            case STREET_RIGHT:
                robot.driveStraightByDistance( robot.DIRECTION_RIGHT, 200, 0.5);
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 300, 0.5);
                break;
            case WALL_LEFT:
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 20, 0.3);
                robot.driveStraightByDistance( robot.DIRECTION_LEFT, 200, 0.5);
                break;
            case WALL_RIGHT:
                robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 20, 0.3);
                robot.driveStraightByDistance( robot.DIRECTION_RIGHT, 200, 0.5);
                break;
        }

    }

}
