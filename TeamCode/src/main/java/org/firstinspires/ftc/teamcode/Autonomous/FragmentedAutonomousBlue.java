package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Library.ColorEnum;
import org.firstinspires.ftc.teamcode.Library.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "FragmentedAutonomousBlue")

public class FragmentedAutonomousBlue extends LinearOpMode {
    Autonomous a = new Autonomous(this, ColorEnum.Blue);

    @Override
    public void runOpMode() {
        a.initServos();

        a.initLiftPosition();

        waitForStart();

        a.stopInitListPosition();

        a.prepareExtender();

        a.driveToQuarry();

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
