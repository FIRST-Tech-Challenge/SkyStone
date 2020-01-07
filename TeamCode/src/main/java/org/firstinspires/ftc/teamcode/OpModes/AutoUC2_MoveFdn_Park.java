package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Autonomous Mode Usecase 2
 * Description : Start on wall in Building Zone, Grip Foundation with arm, and position it into Building Site.
 *              Using Centrifugal force to hold foundation on its father end and rotate to end position.
 *
 * Code asssumes Blue Alliance, and at end can park either on wall side or near the neutral skybridge
 * Steps:
 * Robot starts between A4, A5 such that it can slight in front of skybridge neutral zone floor
 * On start, robot opens wrist to front position
 * Lift Arm to AboveFoundation level
 * Move robot to in between C5 and C6
 * Move forward till Chassis bumber limit switch is pressed.
 * Drop Arm to OnFoundation level
 * Move Robot forward till foundation hits wall
 * Move Robot Left toward A4 (for XX rotations). Friction will cause Robot to rotate towards A6
 * Pull back till wall is hit (Motor does not move)
 * Slide left till Motor does not move (Foundation corner on Edge)
 * Push forward to move foundation to end of line
 * Lift Arm to Above foundation level
 * Move back till wall is hit
 * Move right till outside of foundation
 * (Alternate usecase) : Move forward to align to parking location near neatural sybridge, else stay
 * to park along the wall.
 * Move right by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge
 *
 * Uses playingAlliance variable to select as 1 for Blue, -1 for Red Alliance
 * Uses parkingPlaceNearSkyBridge variable false for near wall, true for near NeutralSkybridge
 */

public class AutoUC2_MoveFdn_Park {

    public int skystonePosition;

    boolean parkedStatus = false;



    /**
     * Template runOpMode code. Only change Usecase function and call here.
     * Refer to Autonomous Command Syntax to put right values
     * <p>
     * All Usecases written assuming playingAlliance = 1 meaning Blue, -1 for Red.
     *
     * throws InterruptedException
     */

    public boolean AutoUC2_MoveFdn_Park_Method(
            LinearOpMode callingOpMode,
            int playingAlliance,
            boolean parkingPlaceNearSkyBridge,
            Chassis autoUCChassis,
            Arm autoUCArm,
            Intake autoUCIntake
    ) {

        //Robot starts between A4, A5 such that it can slight in front of skybridge neutral zone floor

        //On start, Lift arm and robot opens wrist to front position
        //initArm() and initIntake() should do this on class initialization

        //Lift Arm to AboveFoundation level
        autoUCArm.moveArm_aboveFoundationLevel();
        callingOpMode.telemetry.update();

        //Move robot to in between C5 and C6
        double robotToFoundation = 50;
        autoUCChassis.runFwdBackLeftRight(robotToFoundation, playingAlliance,0.25,callingOpMode);

        callingOpMode.sleep(500);
        //Move forward till Chassis bumber limit switch is pressed.
        autoUCChassis.runFwdTill_frontleftChassisTouchSensor_Pressed(7, 0.1, callingOpMode);
        callingOpMode.sleep(500);

        //Drop Arm and Hook to OnFoundation level
        autoUCArm.moveArm_onFoundationLevel();
        autoUCChassis.moveHook_holdFoundation();
        callingOpMode.sleep(500);

        //Move foundation to wall and then turn
        autoUCChassis.runFwdBackLeftRight(6,0,0.25, callingOpMode);
        callingOpMode.sleep(500);

        //Move Robot Left toward A4 (for XX rotations). Friction will cause Robot to rotate towards A6
        double foundationTurnDistance = 83; //was 85
        autoUCChassis.runFwdBackLeftRight(foundationTurnDistance,playingAlliance*(-1),0.25, callingOpMode);
        callingOpMode.sleep(500);

        //Drop Arm and Hook to OnFoundation level
        autoUCArm.moveArm_onFoundationLevel();
        autoUCChassis.moveHook_holdFoundation();
        callingOpMode.sleep(500);

        //Pull back till wall is hit (Motor does not move)
        double foundationBackToWall = 15; // #TOBECORRECTED WITH ENCODER NOT MOVING CODE
        autoUCChassis.runFwdBackLeftRight(-foundationBackToWall,0,0.1, callingOpMode);
        callingOpMode.sleep(100);
        //Slide left till Motor does not move (Foundation corner on Edge)
        //#TOBEWRITTEN

        //Lift Arm to Above foundation level and release hook
        autoUCArm.moveArm_aboveFoundationLevel();
        autoUCChassis.moveHook_Released();
        callingOpMode.sleep(500);

        //Push forward to move foundation to end of line
        double foundationtoEdgeofBuildingSite = 1;
        autoUCChassis.runFwdBackLeftRight(foundationtoEdgeofBuildingSite,0,0.1, callingOpMode);
        callingOpMode.sleep(100);

        //Move back till wall is hit
        autoUCChassis.runFwdBackLeftRight(-4,0,0.25, callingOpMode);
        callingOpMode.sleep(500);

        //Move out of foundation area
        autoUCChassis.runFwdBackLeftRight(30, playingAlliance, 0.25, callingOpMode);

       //Move Arm to ground Level
        autoUCArm.turnArmBrakeModeOn();
        callingOpMode.sleep(500);


        //Optional : Move to park near skybridge Neutral
        if (parkingPlaceNearSkyBridge){
            autoUCChassis.runFwdBackLeftRight(25,0,0.25, callingOpMode);
        }

        //Turn by 90 degrees to point arm forward
        autoUCChassis.turnby90degree(-playingAlliance,0.25, callingOpMode);

        //Park near wall
        //Move right by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge
        if (playingAlliance == 1) {
            //Blue Alliance
            autoUCChassis.runTill_ChassisRightColorSensorIsBlue(-30, 0, 0.25, callingOpMode);
        } else {
            //Red Alliance
            autoUCChassis.runTill_ChassisRightColorSensorIsRed(-30, 0, 0.25, callingOpMode);
        }

        //Reached Parking position
        return parkedStatus = true;
    }
}
