package org.firstinspires.ftc.teamcode.concept;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;

public class ConceptArcAuto extends LinearOpMode {

    MaccabotV2 robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new MaccabotV2(this);
        robot.initialize(true);

        waitForStart();

        while (opModeIsActive()) {
            robot.drive.setTargetsInches(robot.drive.calculateArcLengths(24, 1));
            robot.drive.arcToTargets(2500);
            while (opModeIsActive() && robot.drive.isDriveBusy()) {
                telemetry.update();
            }
            robot.drive.setMotorPowers(0, 0, 0, 0);
            telemetry.addLine("T1");

            robot.drive.setTargetsInches(robot.drive.calculateArcLengths(24, -1));
            robot.drive.arcToTargets(-2500);
            while (opModeIsActive() && robot.drive.isDriveBusy()) {
                telemetry.update();
            }
            robot.drive.setMotorPowers(0, 0, 0, 0);
            telemetry.addLine("T2");
        }
    }
}
