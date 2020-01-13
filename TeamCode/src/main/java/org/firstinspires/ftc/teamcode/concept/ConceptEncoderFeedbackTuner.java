package org.firstinspires.ftc.teamcode.concept;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MaccabotV2;

@Config
@Autonomous(name="Encoder Feedback Tuner", group = "Concept")
//@Disabled
public class ConceptEncoderFeedbackTuner extends LinearOpMode {

    MaccabotV2 robot;

    public static int DISTANCE_INCHES = 24;
    public static int VELOCITY_INCHES = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(true);

        waitForStart();

        telemetry.clearAll();
        telemetry.addLine("FOR TUNING PURPOSES ONLY | 4466 Internal");
        telemetry.addLine();
        while (opModeIsActive()) {
            robot.drive.updateCoefficientsFromConfigutation();
            // Drive forward to target
            robot.drive.setLinearTargetsInches(DISTANCE_INCHES, DISTANCE_INCHES);
            while (opModeIsActive() && robot.drive.isDriveBusy()) {
                robot.drive.runToTargets(VELOCITY_INCHES);
            }
            robot.drive.setMotorPowers(0, 0, 0, 0);
            // Drive in reverse to origin
            robot.drive.setLinearTargetsInches(0, 0);
            while (opModeIsActive() && robot.drive.isDriveBusy()) {
                robot.drive.runToTargets(VELOCITY_INCHES);
            }
            robot.drive.setMotorPowers(0, 0, 0, 0);
        }

    }
}
