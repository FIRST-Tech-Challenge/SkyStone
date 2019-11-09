package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Building Zone Blue")
public class MM_BuildingZoneBlue extends LinearOpMode {

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
        robot.moveWaffleMover('h');

        // go to center of foundation
        robot.driveForwardDistance(6, -speed, this);
        robot.strafeTime(0.5, 1000);
        Thread.sleep(500);

        robot.driveForwardDistance(30, -speed, this);

        // retract the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('h');

        // move backwards a little before turning
        robot.driveForwardDistance(12, 0.25, this);

        // turn 180 degrees
        robot.turnRight(speed, 2500);

        // drive forwards
        robot.driveForwardDistance(36.0, -speed, this);

        // extend the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('f');

        // back up so we have room to turn
        robot.driveForwardDistance(8, 0.25, this);

        // turn towards skybridge
        robot.turnRight(speed, 600);

        // bring arm down
        robot.bringArmDown(this);

        // drive out from foundation
        robot.driveForwardDistance(22, speed, this);
        // if parking close to center, move forward more
        switch (parkingPosition) {
            case CLOSE:
                break;
            case FAR:
                // strafe to the far side
                robot.strafeTime(speed, 1000);
                break;
        }
    }
}


