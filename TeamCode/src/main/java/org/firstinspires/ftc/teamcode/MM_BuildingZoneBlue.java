package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Building Zone Blue")
public class MM_BuildingZoneBlue extends LinearOpMode {

    Robot robot = new Robot();
    enum ParkingPosition {Far, Close}// far or close to center
    ParkingPosition parkingPosition = ParkingPosition.Close;
    double speed = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.driveForwardDistance(47.0 - robot.ROBOT_RETRACTED_LENGTH, speed, this);

        // extend the waffle mover
        wait(500);
        robot.waffleMover.setPower(0.5);
        wait(1000);
        robot.waffleMover.setPower(0);

        // drive backwards
        robot.driveForwardDistance(47.0 - robot.ROBOT_RETRACTED_LENGTH, -speed, this);

        // retract the waffle mover
        wait(500);
        robot.waffleMover.setPower(-0.5);
        wait(1000);
        robot.waffleMover.setPower(0);

        // strafe out from behind the foundation
        robot.strafeTime(-0.25, 3000);

        // drive forward to turn and park under the skybridge
        robot.driveForwardDistance(12.0, speed, this);

        // if parking close to center, move forward more
        if (parkingPosition == ParkingPosition.Close) {
            robot.driveForwardDistance(12.0, speed, this);
        }

        // turn towards skybridge
        robot.turnRight(speed, 1475);

        // park under skybridge
        robot.driveForwardDistance(45.0, speed, this);
    }
}


