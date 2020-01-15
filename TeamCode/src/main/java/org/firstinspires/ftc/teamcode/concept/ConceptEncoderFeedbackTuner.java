package org.firstinspires.ftc.teamcode.concept;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MaccaDrive;
import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;

@Config
@Autonomous(name="Encoder Feedback Tuner", group = "Concept")
//@Disabled
public class ConceptEncoderFeedbackTuner extends LinearOpMode {

    private MaccabotV2 robot;

    public static int DISTANCE_INCHES = 24; // inches
    public static int VELOCITY_INCHES = 60; // inches per second

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MaccabotV2(this);
        robot.initialize(true);

        waitForStart();

        telemetry.clearAll();
        telemetry.addLine("FOR TUNING PURPOSES ONLY | 4466 Internal");
        telemetry.addLine();
        robot.drive.composeTelemetry(MaccaDrive.TelemetryLevel.FULL);
        telemetry.update();

        while (opModeIsActive()) {
            robot.drive.updateCoefficientsFromConfigutation();
            // Drive forward to target
            robot.drive.setTargetsInches(DISTANCE_INCHES, DISTANCE_INCHES);
            telemetry.update();

            while (opModeIsActive() && robot.drive.isDriveBusy()) {
                telemetry.update();
                robot.drive.runToTargetsInches(VELOCITY_INCHES, VELOCITY_INCHES);
            }
            robot.drive.setMotorPowers(0, 0, 0, 0);
            telemetry.addLine("T1");
            telemetry.update();
            robot.drive.updateCoefficientsFromConfigutation();
            // Drive in reverse to origin
            robot.drive.setTargetsInches(-DISTANCE_INCHES, -DISTANCE_INCHES);
            telemetry.update();
            while (opModeIsActive() && robot.drive.isDriveBusy()) {
                telemetry.update();
                robot.drive.runToTargetsInches(VELOCITY_INCHES, VELOCITY_INCHES);
            }
            robot.drive.setMotorPowers(0, 0, 0, 0);
            telemetry.addLine("T2");
            telemetry.update();
        }

    }
}
