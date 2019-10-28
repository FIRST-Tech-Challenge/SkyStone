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
        robot.init(hardwareMap);

        waitForStart();

        robot.driveForwardDistance(47.0 - robot.ROBOT_RETRACTED_LENGTH, -speed, this);

        // extend the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('h');

        // drive backwards
        robot.driveForwardDistance(50.0, 0.75, this);

        // retract the waffle mover
        Thread.sleep(500);
        robot.moveWaffleMover('f');

        // strafe out from behind the foundation
        robot.strafeTime(-0.5, 5000);

        // drive forward to turn and park under the skybridge
        robot.driveForwardDistance(10.0, -speed, this);

        // turn towards skybridge
        robot.turnRight(-speed, 600);

        // if parking close to center, move forward more
        switch (parkingPosition) {
            case CLOSE:
                robot.driveForwardDistance(10.0, -speed, this);
                Thread.sleep(500);
                break;
            case FAR:
                break;
        }

        // park under skybridge
        robot.driveForwardDistance(10.0, -speed, this);
    }
}


