package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad1;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;


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

@TeleOp(name = "NewGripTest", group = "Teleop")
public class NewGripTest extends LinearOpMode{

    public boolean HzDEBUG_FLAG = true;

    HzGamepad1 hzGamepad1;
    Chassis hzChassis;
    Arm hzArm;
    Intake hzIntake;

    //Timer for timing Autonomous mode
    ElapsedTime AutonomousTimeOut = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        hzChassis = new Chassis(hardwareMap);
        hzArm = new Arm(hardwareMap);
        hzIntake = new Intake(hardwareMap);
        hzGamepad1 = new HzGamepad1(gamepad1);

        telemetry.setAutoClear(false);
        telemetry.addData("New Grip Test", "v:1.0");

        //Wait for pressing Run on controller
        waitForStart();
        AutonomousTimeOut.reset();

        //Write test methods as separate functions and call in OpModeisActive().
        //Comment out the ones not run at the time


        while (opModeIsActive()) {
            //If A is pressed, close grip
            if (hzGamepad1.getButtonAPress()){
                //hzIntake.closeGrip();
                hzIntake.left_grip.setPosition(0.5);
                hzIntake.right_grip.setPosition(0.5);
            }
            //If Y is pressed, open grip
            if (hzGamepad1.getButtonYPress()){
                hzIntake.left_grip.setPosition(1);
                hzIntake.right_grip.setPosition(0);
            }

            //If Y is pressed, open grip
            if (hzGamepad1.getButtonBPress()){
                double current_left = hzIntake.left_grip.getPosition();
                hzIntake.left_grip.setPosition(current_left-0.05);
                double current_right = hzIntake.right_grip.getPosition();
                hzIntake.right_grip.setPosition(current_right+0.05);
            }

            //If Y is pressed, open grip
            if (hzGamepad1.getButtonXPress()){
                double current_left = hzIntake.left_grip.getPosition();
                hzIntake.left_grip.setPosition(current_left+0.05);
                double current_right = hzIntake.right_grip.getPosition();
                hzIntake.right_grip.setPosition(current_right-0.05);
            }

            if(HzDEBUG_FLAG) printDebugMessages();
            telemetry.update();
            idle();

        }

    }
    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);
        telemetry.addData("hzIntake.left_grip.getPosition(): ",hzIntake.left_grip.getPosition());
        telemetry.addData("hzIntake.right_grip.getPosition(): ",hzIntake.right_grip.getPosition());


/*        telemetry.addData("hzChassis.backLeft.isBusy : ", hzChassis.backLeft.isBusy());
        telemetry.addData("hzChassis.backLeft.getTargetPosition : ", hzChassis.backLeft.getTargetPosition());
        telemetry.addData("hzChassis.backLeft.getCurrentPosition : ", hzChassis.backLeft.getCurrentPosition());
        telemetry.addData("hzChassis.backRight.getCurrentPosition : ", hzChassis.backRight.getCurrentPosition());
        telemetry.addData("hzChassis.frontLeft.getCurrentPosition : ", hzChassis.frontLeft.getCurrentPosition());
        telemetry.addData("hzChassis.frontRight.getCurrentPosition : ", hzChassis.frontRight.getCurrentPosition());
        telemetry.addData("hzChassis.backLeft.getMode : ", hzChassis.backLeft.getMode());
        //telemetry.addData("hzChassis.frontLeft.currentLevel : ", hzArm.currentLevel);
        //telemetry.addData("Arm.blockLevel[hzArm.currentLevel] : ", hzArm.blockLevel[hzArm.currentLevel]);
*/
    }
}
