package org.firstinspires.ftc.teamcode.concept;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;

@Config
@Autonomous(name="Encoder Feedback Tuner", group = "Concept")
//@Disabled
public class ConceptEncoderFeedbackTuner extends LinearOpMode {

    private MaccabotV2 robot;

    public static int DISTANCE_INCHES = 24;
    public static int VELOCITY_TICKS = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MaccabotV2(this);
        robot.initialize(true);

        waitForStart();

        telemetry.clearAll();
        telemetry.addLine("FOR TUNING PURPOSES ONLY | 4466 Internal");
        telemetry.addLine();
        telemetry.update();
        while (opModeIsActive()) {
            robot.drive.updateCoefficientsFromConfigutation();
            // Drive forward to target
            robot.drive.setTargetsInches(DISTANCE_INCHES, DISTANCE_INCHES);
            while (opModeIsActive() && robot.drive.isDriveBusy()) {
                robot.drive.runToTargets(VELOCITY_TICKS, VELOCITY_TICKS);
            }
            robot.drive.setMotorPowers(0, 0, 0, 0);

            robot.drive.updateCoefficientsFromConfigutation();
            // Drive in reverse to origin
            robot.drive.setTargetsInches(0, 0);
            while (opModeIsActive() && robot.drive.isDriveBusy()) {
                robot.drive.runToTargets(VELOCITY_TICKS, VELOCITY_TICKS);
            }
            robot.drive.setMotorPowers(0, 0, 0, 0);
        }

    }
}
