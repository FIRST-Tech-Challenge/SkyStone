package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Loading Zone Red")
public class MM_BuildingZone extends LinearOpMode {

    Robot robot = new Robot();
    enum ParkingPosition {Far, Close}// far or close to center
    double speed = .4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.driveForwardDistance(47.0 - robot.ROBOT_RETRACT_LENGTH, speed, this);

        wait(500);
        robot.waffleMover.setpower()


    }
}
