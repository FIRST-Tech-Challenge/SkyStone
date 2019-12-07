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
public class AutonomousUC1 extends LinearOpMode {

    public Intake autoIntake;
    public Arm autoArm;
    public Chassis autoChassis;

    public int skystonePosition;

    public int robotDepth = 17; // Ball on wall to Edge of Chassis Touch sensor
    public int robotWidth = 17; // Wheel edge to wheel edge

    public int playingAlliance = 1; //1 for Blue, -1 for Red

    public boolean finalStateAchieved = false; //1 when reached final parking state

    ElapsedTime AutonomousTimeOut = new ElapsedTime();

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
        while (opModeIsActive() && (!finalStateAchieved)) {
            AutonomousUC3Commands();
            finalStateAchieved = true;
        }
    }

    public void AutonomousUC3Commands() {

        // Robot starts on SB5

        //On start, Lift arm and robot opens wrist to front position
        //initArm() and initIntake() should do this on class initialization

        // Lift Arm to Sense Position
        autoArm.moveArm_detectSkystoneLevel();
        if (!opModeIsActive()) return;

        // Move by distance X forward near SB5 : 6 inches to skystone
        double robotToNearSkystone = 20;
        autoChassis.runDistance(robotToNearSkystone, 0, 0, 0.25);
        if (!opModeIsActive()) return;

        // Check on color sensor, for Skystone
        moveTillStoneDetected();

        // If Skystone, record Skystone position as SB5, Go to Step 10

        skystonePosition = 5; // Assume current position is skystone
        double stoneTostone = 8;
        if ((autoIntake.stoneDetected = true) && (autoIntake.skystoneDetected = false)) {
            //Skystone not detected, move to SB4
            skystonePosition = 4;
            autoChassis.runDistance(stoneTostone, playingAlliance * (-Math.PI / 2), 0, 0.1);
        }

        if ((autoIntake.stoneDetected = true) && (autoIntake.skystoneDetected = false)) {
            //Skystone not detected, move to SB3
            skystonePosition = 3;
            autoChassis.runDistance(stoneTostone, playingAlliance * (-Math.PI / 2), 0, 0.1);
        }

        // Drop Arm and Grip the block.
        autoArm.moveArm_groundLevel();
        autoIntake.closeGrip();
        if (!opModeIsActive()) return;

        // Slide back to edge of B2,
        autoChassis.runDistance(-20,0,0,0.25);
        if (!opModeIsActive()) return;

        // Turn 90 degrees Left
        autoChassis.runDistance(0, 0, playingAlliance*Math.PI/2, 0.1);
        if (!opModeIsActive()) return;
        //Lift Arm
        autoArm.moveArm_AutoPlacementLevel();
        if (!opModeIsActive()) return;

       //Move forward till Chassis bumber limit switch is pressed.
        double expectedMaxDistanceToFoundation = 40;
        autoChassis.runFwdTill_frontleftChassisTouchSensor_Pressed(expectedMaxDistanceToFoundation, 0.25);

        // Drop block
        autoIntake.openGrip();
        if (!opModeIsActive()) return;

        // Move in between B4 and B3 (Parking)
        // Park near wall
        // Move back by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge
        if (playingAlliance == 1) {
            autoChassis.runTill_ChassisRightColorSensorIsBlue(-70, 0, 0.25);
        } else {
            autoChassis.runTill_ChassisLeftColorSensorIsRed(-70, 0, 0.25);
        }
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
