package org.firstinspires.ftc.teamcode.concept;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MaccabotV2;

public class ConceptMaccaDriveAuto extends LinearOpMode {

    MaccabotV2 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MaccabotV2(this);
        robot.initialize(true);

        waitForStart();

        telemetry.clearAll();
        telemetry.addLine("OpMode Started.");

        robot.drive.setLinearTargetsTicks(1000, 1000);
        telemetry.addLine("Targets set.");

        while (opModeIsActive() && robot.drive.isDriveBusy()) {
            robot.drive.runToTargets(40);
        }
        robot.drive.setMotorPowers(0, 0, 0, 0);
        telemetry.addLine("Target achieved. All clear!");
        sleep(5000);
    }
}
