package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Thread.*;

@Autonomous(name = "Building Zone Red")
public class MM_BuildingZoneRed_1 extends LinearOpMode {

    private Robot robot = new Robot();
    private enum ParkingPosition {FAR, CLOSE}// far or close to center
    private ParkingPosition parkingPosition = ParkingPosition.CLOSE;
    private double speed = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        // extend the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('h');

        robot.driveForwardDistance(50.0 - robot.ROBOT_RETRACTED_LENGTH, -speed, this);

        // retract the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('h');

        // drive backwards
        robot.driveForwardDistance(50.0, 0.25, this);

        // extend the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('f');

        // strafe out from behind the foundation
        robot.strafeTime(0.5, 5000);

        // drive forward to turn and park under the skybridge
        robot.driveForwardDistance(10.0, -speed, this);

        // if parking close to center, move forward more
        switch (parkingPosition) {
            case CLOSE:
                robot.driveForwardDistance(10.0, -speed, this);
                Thread.sleep(500);
                break;
            case FAR:
                break;
        }

        // turn towards skybridge
        robot.turnRight(speed, 600);

        // park under skybridge
        robot.driveForwardDistance(10.0, -speed, this);
    }
}


