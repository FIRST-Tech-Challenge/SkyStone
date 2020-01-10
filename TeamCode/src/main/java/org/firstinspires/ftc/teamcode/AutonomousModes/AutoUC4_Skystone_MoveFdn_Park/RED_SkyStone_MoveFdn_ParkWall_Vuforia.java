package org.firstinspires.ftc.teamcode.AutonomousModes.AutoUC4_Skystone_MoveFdn_Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.AutoUC4_Skystone_MoveFdn_Park;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Autonomous Mode Usecase 4
 *
 * Description : Start on wall in Loading Zone, identify and move 1 skystone to Building zone
 *                  on foundation, pull foundation to Building zone and
 *                  park near wall or near neutral Skybridge
 *
 *  Uses playingAlliance variable to select as 1 for Blue, -1 for Red Alliance
 *  Uses parkingPlaceNearSkyBridge variable false for near wall, true for near NeutralSkybridge
 */

@Autonomous(name = "RED-SkyStone-MoveFdn_ParkWall-Vuforia", group = "Skystone-MoveFdn")
public class RED_SkyStone_MoveFdn_ParkWall_Vuforia extends LinearOpMode {

    Intake autoIntake;
    Arm autoArm;
    Chassis autoChassis;

    int playingAlliance = -1; //1 for Blue, -1 for Red
    boolean parkingPlaceNearSkyBridge = false;//false for near wall, true for near NeutralSkybridge

    boolean parked = false; // Will be true once robot is parked

    AutoUC4_Skystone_MoveFdn_Park AutoMode;

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
        AutoMode = new AutoUC4_Skystone_MoveFdn_Park();
        //telemetry.setAutoClear(false);

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
            parked = AutoMode.AutoUC4_Skystone_MoveFdn_Park_Method(
                    this,
                    playingAlliance,
                    parkingPlaceNearSkyBridge,
                    autoChassis,
                    autoArm,
                    autoIntake);
        }
    }
}
