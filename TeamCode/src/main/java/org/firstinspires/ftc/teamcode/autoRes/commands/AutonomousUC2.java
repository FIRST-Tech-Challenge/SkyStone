package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Autonomous Mode Usecase 2
 * Description : Start on wall in Building Zone, Grip Foundation with hook, and position it into Building Site.
 *
 * Steps:
 * Robot starts between A4, A5 such that it can slight in front of skybridge neutral zone floor
 * On start, robot opens wrist to front position
 * Lift Arm to AboveFoundation level
 * Move robot to in between C5 and C6
 * Move forward till Chassis bumber limit swich is pressed.
 * Drop Arm to OnFoundation level
 * Move Robot Left toward A4 (for XX rotations). Friction will cause Robot to rotate towards A6
 * Pull back till wall is hit (Motor does not move)
 * Slide left till Motor does not move (Foundation corner on Edge)
 * Push forward to move foundation to end of line
 * Lift Arm to Above foundation level
 * Move back till wall is hit
 * Move right by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge
 */

@Autonomous(name = "AutoUseCase2", group = "Autonomous")
public class AutonomousUC2 extends LinearOpMode {

    Intake autoIntake;
    Arm autoArm;
    Chassis autoChassis;

    public int skystonePosition;

    public int robotDepth = 17; // Ball on wall to Edge of Chassis Touch sensor
    public int robotWidth = 17; // Wheel edge to wheel edge

    int playingAlliance = 1; //1 for Blue, -1 for Red

    boolean parked = false; // Will be true once robot is parked

    ElapsedTime AutonomousTimeOut = new ElapsedTime();

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

        autoIntake = new Intake(hardwareMap);
        autoArm = new Arm(hardwareMap);
        autoChassis = new Chassis(hardwareMap);

        //Robot starts on A2
        waitForStart();

        //Initialize on press of play
        autoChassis.initChassis();
        autoArm.initArm();
        autoIntake.initIntake();

        while (opModeIsActive() && !parked) {
            AutonomousUC2Commands();
        }
    }

    void AutonomousUC2Commands() {

        //Robot starts between A4, A5 such that it can slight in front of skybridge neutral zone floor

        //On start, Lift arm and robot opens wrist to front position
        //initArm() and initIntake() should do this on class initialization

        //Lift Arm to AboveFoundation level
        autoArm.moveArm_aboveFoundationLevel();

        //Move robot to in between C5 and C6
        double robotToFoundation = 39.75; //47.5 - robotWidth + 18.5 / 2;
        autoChassis.runFwdBackLeftRight(playingAlliance*robotToFoundation,1,0.25);
        //Testing : Moved 54" vs 39.75" input


        //Move forward till Chassis bumber limit switch is pressed.
        autoChassis.runFwdTill_frontleftChassisTouchSensor_Pressed(4, 0.1);
        //Testing : Move 6.5" vs 4" input, stopped correctly when touch sensor is pressed

        //Drop Arm to OnFoundation level
        autoArm.moveArm_onFoundationLevel();

        //Optional : Move foundation to wall and then turn

        //Move Robot Left toward A4 (for XX rotations). Friction will cause Robot to rotate towards A6
        double foundationTurnDisance = 39.75;
        autoChassis.runFwdBackLeftRight(playingAlliance*foundationTurnDisance,-1,0.25);

        //Pull back till wall is hit (Motor does not move)
        double foundationBackToWall = 10; // #TOBECORRECTED WITH ENCODER NOT MOVING CODE
        autoChassis.runFwdBackLeftRight(-foundationBackToWall,0,0.25);

        //Slide left till Motor does not move (Foundation corner on Edge)
        //#TOBEWRITTEN

        //Push forward to move foundation to end of line
        double foundationtoEdgeofBuildingSite = 3;
        autoChassis.runFwdBackLeftRight(foundationtoEdgeofBuildingSite,0,0.25);

        //Lift Arm to Above foundation level
        autoArm.moveArm_aboveFoundationLevel();

        //Move back till wall is hit
        autoChassis.runFwdBackLeftRight(-foundationtoEdgeofBuildingSite,0,0.25);


        //Optional : Move to park near skybridge Neutral

        //Park near wall
        //Move right by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge
        if (playingAlliance == 1) {
            //Blue Alliance
            //autoChassis.runTill_ChassisRightColorSensorIsBlue(70, 1, 0.25);
            autoChassis.runTill_ChassisRightColorSensorIsRed(55, 1, 0.25);
        } else {
            //Red Alliance
            autoChassis.runTill_ChassisLeftColorSensorIsRed(70, -1, 0.25);
        }

        //Reached Parking position
        parked = true;
    }

}
