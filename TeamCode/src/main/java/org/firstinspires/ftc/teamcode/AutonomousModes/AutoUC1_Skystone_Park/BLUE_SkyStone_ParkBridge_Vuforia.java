package org.firstinspires.ftc.teamcode.AutonomousModes.AutoUC1_Skystone_Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.AutoUC1_Skystone_Park;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Autonomous Mode Usecase 1
 *
 * Description : Start on wall in Loading Zone, identify and move 1 skystone to Building zone and
 *                  park near wall or near neutral Skybridge
 *
 *  Uses playingAlliance variable to select as 1 for Blue, -1 for Red Alliance
 *  Uses parkingPlaceNearSkyBridge variable false for near wall, true for near NeutralSkybridge
 */

@Autonomous(name = "BLUE-SkyStone-ParkBridge-Vuforia", group = "Skystone")
public class BLUE_SkyStone_ParkBridge_Vuforia extends LinearOpMode {

    Intake autoIntake;
    Arm autoArm;
    Chassis autoChassis;

    int playingAlliance = 1; //1 for Blue, -1 for Red
    boolean parkingPlaceNearSkyBridge = true;//false for near wall, true for near NeutralSkybridge

    boolean parked = false; // Will be true once robot is parked

    AutoUC1_Skystone_Park AutoMode;

    /**
     * Template runOpMode code. Only change Usecase function and call here.
     * Refer to Autonomous Command Syntax to put right values
     * <p>
     * All Usecases written assuming playingAlliance = 1 meaning Blue, -1 for Red.
     *
     * throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        AutoMode = new AutoUC1_Skystone_Park();
        telemetry.setAutoClear(false);

        autoIntake = new Intake(hardwareMap);
        autoArm = new Arm(hardwareMap);
        autoChassis = new Chassis(hardwareMap);

        AutoMode.vuforiaInit(hardwareMap);

        waitForStart();

        //Initialize on press of play
        autoChassis.initChassis();
        autoArm.initArm();
        autoIntake.initIntake();

        while (opModeIsActive()&& !isStopRequested() && !parked) {
            parked = AutoMode.AutoUC1_Skystone_Park_Method(
                    this,
                    playingAlliance,
                    parkingPlaceNearSkyBridge,
                    autoChassis,
                    autoArm,
                    autoIntake);
        }
    }
}
