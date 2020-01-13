package org.firstinspires.ftc.teamcode.AutonomousModes.AutoUC3_Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.AutoUC3_Park;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Autonomous Mode Usecase 3 <BR>
 * Description : Start on wall in Building Zone / Loading Zone ,
 *              Park near wall or near neutral Skybridge <BR>
 * <BR>
 * Uses playingAlliance variable to select as 1 for Blue, -1 for Red Alliance <BR>
 * Uses parkingPlaceNearSkyBridge variable false for near wall, true for near NeutralSkybridge <BR>
 * Uses startInBuildingZone variable as true for building zone, false for loading zone <BR>
 *
 */

@Autonomous(name = "RED-ParkBridge_fromBuildingZone", group = "Park")
public class RED_ParkBridge_fromBuildingZone extends LinearOpMode {

    Intake autoIntake;
    Arm autoArm;
    Chassis autoChassis;

    int playingAlliance = -1; //1 for Blue, -1 for Red
    boolean startInBuildingZone = true; // true for building zone, false for loading zone
    boolean parkingPlaceNearSkyBridge = true;//false for near wall, true for near NeutralSkybridge

    AutoUC3_Park AutoMode;

    boolean parked = false; // Will be true once robot is parked

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

        AutoMode = new AutoUC3_Park();
        autoIntake = new Intake(hardwareMap);
        autoArm = new Arm(hardwareMap);
        autoChassis = new Chassis(hardwareMap);

        waitForStart();

        //Initialize on press of play
        autoChassis.initChassis();
        autoArm.initArm();
        autoIntake.initIntake();

        while (opModeIsActive()&& !isStopRequested() && !parked) {
            parked = AutoMode.AutoUC3_Park_Method(
                    this,
                    playingAlliance,
                    parkingPlaceNearSkyBridge,
                    startInBuildingZone,
                    autoChassis,
                    autoArm,
                    autoIntake);
        }
    }
}