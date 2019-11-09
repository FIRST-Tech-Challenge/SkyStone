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

        robot.driveForwardDistance(22, -0.25, this);

        // retract the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('h');
        Thread.sleep(500);

        // before turning, back up a little so we don't run into the other team's robot
        robot.driveForwardDistance(12, 0.25, this);

        // turn 180 degrees
        robot.turnRight(0.6, 3200);
    
        // drive backwards
        robot.driveForwardDistance(30.0, -0.6, this);

        // extend the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('f');

        // back up from the foundation
        robot.driveForwardDistance(6, speed, this);

        // turn towards skybridge
        robot.turnRight(speed, 1250);

        // drive forward to turn and park under the skybridge
        robot.driveForwardDistance(8, speed, this);

        // if parking close to center, move forward more
        switch (parkingPosition) {
            case CLOSE:
                break;
            case FAR:
                robot.strafeTime(speed, 1500);
                break;
        }

        // extend arm so we are under 14 inches
        //robot.toggleArmRotate();

        // park under skybridge
        robot.driveForwardDistance(15.0, speed, this);
    }
}


