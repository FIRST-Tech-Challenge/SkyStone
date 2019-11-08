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

        robot.driveForwardDistance(36, -speed, this);

        // retract the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('h');

        // turn 180 degrees
        robot.turnRight(speed, 2500);

        // drive backwards
        robot.driveForwardDistance(50.0, -0.25, this);

        // extend the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('f');

        // strafe out from behind the foundation
        robot.strafeTime(0.5, 1000);

        // drive forward to turn and park under the skybridge
        robot.driveForwardDistance(5.0, -speed, this);

        // if parking close to center, move forward more
        switch (parkingPosition) {
            case CLOSE:
                robot.driveForwardDistance(15.0, -speed, this);
                Thread.sleep(500);
                break;
            case FAR:
                break;
        }

        // turn towards skybridge
        robot.turnRight(-speed, 600);

        // extend arm so we are under 14 inches
        //robot.toggleArmRotate();

        // park under skybridge
        robot.driveForwardDistance(10.0, -speed, this);
    }
}


