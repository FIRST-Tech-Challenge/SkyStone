package org.firstinspires.ftc.teamcode.concept;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.hardware.MaccaDrive;
import org.firstinspires.ftc.teamcode.hardware.MaccabotV2;

@Autonomous(name="ConceptMaccaDriveAuto")
//@Disabled
public class ConceptMaccaDriveAuto extends LinearOpMode {

    private MaccabotV2 robot;

    @Override
    public void runOpMode() throws InterruptedException {

        // Micah's greatest contribution to the code
        MediaPlayer mp = MediaPlayer.create(hardwareMap.appContext, R.raw.poortown_scrub_noah_cut);

        robot = new MaccabotV2(this);
        robot.initialize(true);
        telemetry.update();
        mp.start();

        waitForStart();

        mp.stop();

        telemetry.clearAll();
        telemetry.addLine("OpMode Started.");
        robot.drive.composeTelemetry(MaccaDrive.TelemetryLevel.FULL);
        telemetry.update();

        robot.drive.setTargetsTicks(1000, 1000);
        telemetry.addLine("Targets set.");
        telemetry.update();
        while (opModeIsActive() && robot.drive.isDriveBusy()) {
            telemetry.update();
            robot.drive.runToTargets(1500, 1500);
        }
        robot.drive.setMotorPowers(0, 0, 0, 0);
        telemetry.addLine("Target achieved. All clear!");
        telemetry.update();
        sleep(5000);
    }
}
