package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Autonomous Mode Usecase 1
 *
 * Description : Start on wall in Loading Zone, identify and move 1 skystone to Building zone and park next to skybridge
 *
 * Steps:
 * Robot starts on SB5
 * On start, robot opens wrist to front position
 * Lift Arm to Sense Position
 * Move by distance X forward near SB5
 * Check on color sensor, for Skystone
 * If Skystone, record Skystone position as SB5, Go to Step 10
 * Else move robot to SB4. Check on color sensor for Skystone.
 * If Skystone, record Skystone position as SB4, Go to Step 10
 * Else move robot to SB3. record skystone position as SB3.
 * Grip and pick the block.
 * Lift Arm to Level 2 Tray Height
 * Slide back to edge of B2,
 * Turn 90 degrees Left
 * Move to B4
 * Drop block
 * Move in between B4 and B3 (Parking)
 *
 *
 */

@Autonomous(name = "AutoUseCase1", group = "Autonomous")
public class AutonomousTester extends LinearOpMode {

    public Intake autoIntake;
    public Arm autoArm;
    public Chassis autoChassis;

    public int skystonePosition;

    public double robotDepth = 15.7; // Ball on wall to Edge of Chassis Touch sensor
    public double robotWidth = 17.0; // Wheel edge to wheel edge

    public int playingAlliance = 1; //1 for Blue, -1 for Red

    public boolean finalStateAchieved = false; //1 when reached final parking state

    ElapsedTime AutonomousTimeOut = new ElapsedTime();

    public boolean parked = false;

    /**
     * Template runOpMode code. Only change Usecase function and call here.
     * Refer to Autonomous Command Syntax to put right values
     * <p>
     * All Usecases written assuming playingAlliance = 1 meaning Blue, -1 for Red.
     *
     * @throws InterruptedException
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
            AutonomousTester();
        }

    }

    public void AutonomousTester() {

        //Robot starts between A4, A5 such that it can slight in front of skybridge neutral zone floor

        //On start, Lift arm and robot opens wrist to front position
        //initArm() and initIntake() should do this on class initialization

        //Lift Arm to AboveFoundation level
        autoArm.moveArm_aboveFoundationLevel();

        //Move robot to in between C5 and C6
        // istance (+ for forward - for backward), strafe right : direction = -Math.PI/2, strafe left : direction = Math.PI/2
        //distance calibaration forward/backward : 9.8" vs 10" input. strafe left/right : 9" vs 10" input 13" vs 15" input
        //robotToFoundation = wall to Foundation (47.5) - bredth of robot + half of foundation (18.5/2)

        double robotToFoundation = 47.5 - robotWidth + 18.5 / 2;
        //autoChassis.runDistance(robotToFoundation, playingAlliance * (-Math.PI / 2), 0, 0.25);
        //while (autoChassis.backLeft.isBusy());

        //Go right
        autoChassis.runFwdBackLeftRight(robotToFoundation,1,0.25);

        //Move forward till Chassis bumber limit switch is pressed.
        autoChassis.runFwdTill_frontleftChassisTouchSensor_Pressed(4, 0.1);
        //Testing : Move 6.5" vs 4" input, stopped correctly when touch sensor is pressed

        //Drop Arm to OnFoundation level
        autoArm.moveArm_onFoundationLevel();

        //Move Robot Left toward A4 (for XX rotations). Friction will cause Robot to rotate towards A6
        double foundationTurnDisance = 39.75;
        autoChassis.runFwdBackLeftRight(foundationTurnDisance,-1,0.25);
        //autoChassis.runDistance(foundationTurnDisance, playingAlliance * (Math.PI / 2), 0, 0.25);

        //Pull back till wall is hit (Motor does not move)
        double foundationBackToWall = 10; // #TOBECORRECTED WITH ENCODER NOT MOVING CODE
        autoChassis.runFwdBackLeftRight(-foundationBackToWall,0,0.25);
        //autoChassis.runDistance(-foundationBackToWall, 0, 0, 0.25);

        autoArm.moveArm_onFoundationLevel();

        parked = true;

        //End of Usecase : Should be parked at this time.
    }


    /**
     * Method to move till Skystone is detected using color sensor and distance sensor
     */
    public void moveTillStoneDetected(){
        //public void runTill_ChassisLeftColorSensorIsBlue(double max_stop_distance, double straveDirection, double power){

        double stoneDetect_max_stop_distance = 5; //max is 6"
        autoChassis.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        autoChassis.resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = stoneDetect_max_stop_distance/(2*Math.PI*autoChassis.wheelRadius);

        while (!autoIntake.detectSkytoneAndType() && (Math.abs(autoChassis.backLeft.getCurrentPosition()) < Math.abs(autoChassis.ChassisMotorEncoderCount * targetRotations))) {
            autoChassis.frontLeft.setPower(0.1);
            autoChassis.frontRight.setPower(0.1);
            autoChassis.backLeft.setPower(0.1);
            autoChassis.backRight.setPower(0.1);
        }

        autoChassis.setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        autoChassis.frontLeft.setPower(0.0);
        autoChassis.frontRight.setPower(0.0);
        autoChassis.backLeft.setPower(0.0);
        autoChassis.backRight.setPower(0.0);

    } //return stone detected autoIntake.stoneDetected and if skystone autoIntake.SkystoneDetected
}
