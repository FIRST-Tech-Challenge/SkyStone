package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.SubSystems.*;


/**
 * TeleOpMode for testing Autonomous mode functionality
 * Write test methods as separate functions and call in OpModeisActive().
 * Comment out the ones not run at the time, but dont delete the test methods written.
 * Autonomous commands :
 * @ChassisAutoMethods : runDistance()
 * @ChassisAutoMethods : runTill_frontleftBumperSensor_Pressed(max stop distance)
 * @ChassisAutoMethods : runTill_chassisLocationSensorIdentifiesLine(color)
 * @ChassisAutoMethods : turnRobotByAngle()
 * @ChassisAutoMethods : resetColorSensorEnabled()
 * @ChassisAutoMethods : leftColorSensorIsRed()
 * @ChassisAutoMethods : rightColorSensorIsBlue()
 * @ChassisAutoMethods : leftColorSensorIsBlue()
 * @ChassisAutoMethods : rightColorSensorIsRed()
 * @ChassisAutoMethods : frontleftBumperSensorIsPressed()
 * @ArmMethods : moveArm_groundLevel()
 * @ArmMethods : moveArm_blockLevelUp()
 * @ArmMethods : moveArm_blockLevelDown()
 * @ArmMethods : moveArmToPlaceBlockAtLevel()
 * @ArmMethods : moveArmToLiftAfterBlockPlacement()
 * @ArmMethods : runArmToLevel()
 * @ArmAutoMethods : moveArm_detectSkystoneLevel()
 * @ArmAutoMethods : moveArm_aboveFoundationLevel(()
 * @ArmAutoMethods : moveArm_onFoundationLevel()
 * @IntakeMethods : moveWristToClose()
 * @IntakeMethods : moveWristToHorizontal()
 * @IntakeMethods : moveWristToVertical()
 * @IntakeAutoMethods : openGrip()
 * @IntakeAutoMethods : closeGrip()
 * @IntakeAutoMethods : detectSkystoneAndType()
 * @IntakeAutoMethods : detectSkystoneColor()
 * @IntakeAutoMethods : detectSkystoneDistance()
 */

@Autonomous(name = "HzAutoRunDistance", group = "Autonomous")
public class AutoTestRunDistance extends LinearOpMode{

    public boolean HzDEBUG_FLAG = true;

    HzGamepad1 hzGamepad1;
    Chassis autoChassis;
    Arm autoArm;
    Intake autoIntake;
    public int robotDepth = 17; // Ball on wall to Edge of Chassis Touch sensor
    public int robotWidth = 17; // Wheel edge to wheel edge

    public int playingAlliance = 1; //1 for Blue, -1 for Red

    public boolean finalStateAchieved = false; //1 when reached final parking state

    //Timer for timing Autonomous mode
    ElapsedTime AutonomousTimeOut = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        autoChassis = new Chassis(hardwareMap);
        autoArm = new Arm(hardwareMap);
        autoIntake = new Intake(hardwareMap);
        hzGamepad1 = new HzGamepad1(gamepad1);

        telemetry.setAutoClear(false);
        telemetry.addData("Init Autonomous Tests", "v:1.0");

        //Wait for pressing Run on controller
        //Initialize on press of play
        autoChassis.initChassis();
        autoArm.initArm();
        autoIntake.initIntake();

        waitForStart();
        
        //AutonomousTimeOut.reset();

        //Write test methods as separate functions and call in OpModeisActive().
        //Comment out the ones not run at the time

        //Run Robot till opMode is Active or 30 seconds

            //hzGamepad1.runSubsystemByGamepadInput(autoChassis, autoArm, autoIntake);
            //autoArm.moveArm_aboveFoundationLevel();

        autoChassis.runDistance(9, -Math.PI/2, 0, 0.25);// Strave right
        //autoChassis.runTill_ChassisLeftColorSensorIsRed(9, -1, 0.25);
        //autoChassis.runDistance(0, 0, Math.PI/4, 0.25);

        double robotToFoundation = 47.5 - robotWidth + 18.5 / 2;
        autoChassis.runDistance(robotToFoundation, playingAlliance * (-Math.PI / 2), 0, 0.25);


        if(HzDEBUG_FLAG) printDebugMessages();
             telemetry.update();

    }
    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);

        telemetry.addData("autoChassis.backLeft.isBusy : ", autoChassis.backLeft.isBusy());
        telemetry.addData("autoChassis.backLeft.getTargetPosition : ", autoChassis.backLeft.getTargetPosition());
        telemetry.addData("autoChassis.backLeft.getCurrentPosition : ", autoChassis.backLeft.getCurrentPosition());
        telemetry.addData("autoChassis.backRight.getCurrentPosition : ", autoChassis.backRight.getCurrentPosition());
        telemetry.addData("autoChassis.frontLeft.getCurrentPosition : ", autoChassis.frontLeft.getCurrentPosition());
        telemetry.addData("autoChassis.frontRight.getCurrentPosition : ", autoChassis.frontRight.getCurrentPosition());
        telemetry.addData("autoChassis.backLeft.getMode : ", autoChassis.backLeft.getMode());
        //telemetry.addData("autoChassis.frontLeft.currentLevel : ", autoArm.currentLevel);
        //telemetry.addData("Arm.blockLevel[autoArm.currentLevel] : ", autoArm.blockLevel[autoArm.currentLevel]);

    }
}
