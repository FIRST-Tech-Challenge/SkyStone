package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.*;

/**
 * TeleOpMode for testing Autonomous mode functionality
 * Write test methods as separate functions and call in OpModeisActive().
 * Comment out the ones not run at the time, but dont delete the test methods written.
 * Autonomous commands :
 *  * @ChassisAutoMethods : runDistance()
 *  * @ChassisAutoMethods : runTill_frontleftBumperSensor_Pressed(max stop distance)
 *  * @ChassisAutoMethods : runTill_chassisLocationSensorIdentifiesLine(color)
 *  * @ChassisAutoMethods : turnRobotByAngle()
 *  * @ChassisAutoMethods : resetColorSensorEnabled()
 *  * @ChassisAutoMethods : leftColorSensorIsRed()
 *  * @ChassisAutoMethods : rightColorSensorIsBlue()
 *  * @ChassisAutoMethods : leftColorSensorIsBlue()
 *  * @ChassisAutoMethods : rightColorSensorIsRed()
 *  * @ChassisAutoMethods : frontleftBumperSensorIsPressed()
 *  * @ArmMethods : moveArm_groundLevel()
 *  * @ArmMethods : moveArm_blockLevelUp()
 *  * @ArmMethods : moveArm_blockLevelDown()
 *  * @ArmMethods : moveArmToPlaceBlockAtLevel()
 *  * @ArmMethods : moveArmToLiftAfterBlockPlacement()
 *  * @ArmMethods : runArmToLevel()
 *  * @ArmAutoMethods : moveArm_detectSkystoneLevel()
 *  * @ArmAutoMethods : moveArm_aboveFoundationLevel(()
 *  * @ArmAutoMethods : moveArm_onFoundationLevel()
 *  * @IntakeMethods : moveWristToInitialPosition()
 *  * @IntakeMethods : moveWristToHorizontalPosition()
 *  * @IntakeMethods : moveWristToVerticalPosition()
 *  * @IntakeAutoMethods : openGrip()
 *  * @IntakeAutoMethods : closeGrip()
 *  * @IntakeAutoMethods : detectSkystoneColorSensorIsYellow()
 *  * @IntakeAutoMethods : detectSkystoneColorSensorIsBlack()
 */

@Disabled
@TeleOp(name = "HzAutoTest", group = "Teleop")
public class AutonomousTests extends LinearOpMode{

    HzGamepad1 hzGamepad1;

    @Override
    public void runOpMode() {
        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        Chassis hzChassis = new Chassis(hardwareMap);
        Arm hzArm = new Arm(hardwareMap);
        Intake hzIntake = new Intake(hardwareMap);
        hzGamepad1 = new HzGamepad1(gamepad1);

        //Initialize Subsystems - Chassis, Arm, Intake.
        hzChassis.initChassis();
        hzArm.initArm();
        hzIntake.initIntake();

        telemetry.setAutoClear(false);
        telemetry.addData("Init Autonomous Tests", "v:1.0");

        //Wait for pressing Run on controller
        waitForStart();

        //Write test methods as separate functions and call in OpModeisActive().
        //Comment out the ones not run at the time

        //Run Robot
        while (opModeIsActive()) {

            //telemetry.addData("Intake.detectSkystone.Red ", hzIntake.detectSkystone.red() );

            telemetry.update();

        }

    }

}
