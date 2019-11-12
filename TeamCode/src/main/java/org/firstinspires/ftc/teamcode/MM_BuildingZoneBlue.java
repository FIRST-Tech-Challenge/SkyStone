package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Building Zone Blue")
public class MM_BuildingZoneBlue extends LinearOpMode {
    private Robot robot = new Robot();
    private enum ParkingPosition {FAR, CLOSE}// far or close to center
    private ParkingPosition parkingPosition = ParkingPosition.FAR;
    private double speed = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        waitForStart();
        robot.driveForwardDistance(48, -speed, this);
        robot.strafeTime(speed, 2000);
        robot.strafeTime(-speed, 250);
        robot.driveForwardDistance(18, speed, this);
        robot.moveWaffleMover();
        robot.strafeTime(speed, 3000);
        // correction for strafe
        robot.turnRight(0.25, 300);
        robot.driveForwardDistance(8, -0.25, this);
        robot.moveWaffleMover();
        robot.driveForwardDistance(34, 0.25, this);
        robot.moveWaffleMover();
        robot.strafeTime(-speed, 3500);
        robot.driveForwardDistance(6, -0.25, this);
        robot.turnRight(-speed, 1500);
        robot.driveForwardDistance(24, speed, this);
        if (parkingPosition == ParkingPosition.CLOSE) {
            robot.strafeTime(speed, 1000);
        }
    }
}
