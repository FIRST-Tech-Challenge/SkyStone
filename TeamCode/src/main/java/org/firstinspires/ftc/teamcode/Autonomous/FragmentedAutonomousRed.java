package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Library.ColorEnum;
import org.firstinspires.ftc.teamcode.Library.Autonomous;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "FragmentedAutonomousRed")

public class FragmentedAutonomousRed extends LinearOpMode {
    Autonomous a = new Autonomous(this, ColorEnum.Red);

    @Override
    public void runOpMode() {
        a.initServos();

        a.initLiftPosition();

        waitForStart();

        a.stopInitLiftPosition();

        a.prepareExtender();

        a.driveToRandomStone();

        a.grabQuarryStone();

        a.driveBackToWall();

        a.driveToBridge();

        a.driveToBuildingSite();

        a.prepareArmForFoundation();

        a.driveToFoundation();

        if (opModeIsActive()) {
            a.generalTools.openClamp();
            a.generalTools.grabFoundation();
            a.generalTools.stopForMilliSeconds(500);
        }

        a.driveBackToWall();

        if (opModeIsActive()) {
            a.generalTools.releaseFoundation();
        }

        a.driveTowardsBridgeFromBuildingSite();

        a.foldIn();

        a.parkUnderBridge();
    }
}
