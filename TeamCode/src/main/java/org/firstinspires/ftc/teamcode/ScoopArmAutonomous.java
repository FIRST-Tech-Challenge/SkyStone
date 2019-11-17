package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Scoop Arm Auto", group="Exercises")

public class ScoopArmAutonomous extends LinearOpMode {
    private ScoopArmBot robot = new ScoopArmBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            robot.scoopStone();

        }
    }
}
