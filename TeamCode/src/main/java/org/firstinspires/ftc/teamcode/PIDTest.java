package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="PID Straight Test", group="Exercises")

public class PIDTest extends LinearOpMode {
    PIDBot robot = new PIDBot(this);


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive());
            robot.checkDirection();


        robot.driveStraightbyDistancePID(robot.DIRECTION_FORWARD, 30004, 0.3);
        sleep(500);

        robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 1000, 0.3);
        robot.driveStraightByDistance(robot.DIRECTION_FORWARD, 0, 0);
        robot.opMode.sleep(9000);

        robot.rotate(90, 0.4);



    }

}