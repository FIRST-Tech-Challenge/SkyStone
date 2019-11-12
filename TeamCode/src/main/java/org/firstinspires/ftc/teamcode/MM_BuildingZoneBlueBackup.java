package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Building Zone Blue")
public class MM_BuildingZoneBlueBackup extends LinearOpMode {

    private Robot robot = new Robot();
    private enum ParkingPosition {FAR, CLOSE}// far or close to center
    private ParkingPosition parkingPosition = ParkingPosition.CLOSE;
    private double speed = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);

        waitForStart();

        // extend the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover();

        // go to center of foundation
        robot.driveForwardDistance(6, -speed, this);
        robot.strafeTime(0.5, 1250);
        Thread.sleep(500);

        robot.driveForwardDistance(28, -speed, this);

        // retract the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover();

        // move backwards a little before turning
        robot.driveForwardDistance(12, 0.25, this);

        // turn 180 degrees
        robot.turnRight(speed, 5000);

        // drive forwards
        robot.driveForwardDistance(36.0, -speed, this);

        // extend the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover();

        // back up so we have room to turn
        robot.driveForwardDistance(8, 0.25, this);

        // turn towards skybridge
        robot.turnRight(speed, 1200);

        // drive out from foundation
        robot.driveForwardDistance(22, speed, this);
        // if parking close to center, move forward more
        if (parkingPosition == ParkingPosition.FAR) {
            // strafe to the far side
            robot.strafeTime(speed, 1500);
        }
    }
}


