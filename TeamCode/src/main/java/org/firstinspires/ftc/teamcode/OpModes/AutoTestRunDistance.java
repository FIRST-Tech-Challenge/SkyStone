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

    public boolean parked = false;

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

        while (opModeIsActive() && !parked) {
            //chassisFwdBackTest();
            //chassisLeftRightTest();
            autoChassis.turnby90degree(1,0.1);
            parked = true;
        }

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

        telemetry.addData("autoChassis.backLeft.getMode : ", autoChassis.target90degRotations);

    }

    public void chassisFwdBackTest(){
        double testdistance[] = {
                2,
                3,
                4,
                5,
                6,/*
                10,
                15,
                20,
                25,
                30,
                35,
                40,
                45,
                50*/
        };

        int i;
        for (i = 0; i < testdistance.length; i++){
            autoChassis.runFwdBackLeftRight(testdistance[i], 0, 0.1);
            sleep(3000);
            autoChassis.runFwdBackLeftRight(-testdistance[i], 0, 0.1);
            sleep(3000);
        }
        parked = true;

    }

    public void chassisLeftRightTest(){
        double testdistance[] = {
                2,
                3,
                4,
                5,
                6,
                10,
                15,
                20,
                25,
                30,
                35,
                40,
                45,
                50
        };

        int i;
        for (i = 0; i < testdistance.length; i++){
            autoChassis.runFwdBackLeftRight(testdistance[i], 1, 0.25);
            sleep(3000);
            autoChassis.runFwdBackLeftRight(testdistance[i], -1, 0.25);
            sleep(3000);
        }
        parked = true;
    }
}
