package org.firstinspires.ftc.teamcode.UnitTesting;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepad1;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * TeleOpMode for team Hazmat
 */
@Disabled
@TeleOp(name = "NewArmConfiguration", group = "TeleopTest")
public class NewArmConfiguration extends LinearOpMode{

    public boolean HzDEBUG_FLAG = true;

    int hzRobotNum; //Set to 1 for main robot, 2 for second robot

    HzGamepad1 hzGamepad1;
    Chassis hzChassis;
    Arm hzArm;
    Intake hzIntake;

    int currentLevel = 0;

    @Override
    public void runOpMode() {

        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        hzChassis = new Chassis(hardwareMap);
        hzArm = new Arm(hardwareMap);
        hzIntake = new Intake(hardwareMap);
        hzGamepad1 = new HzGamepad1(gamepad1);

        telemetry.addData("NewArmConfiguration", "v:1.0");

        //Wait for pressing plan on controller
        waitForStart();

        //Initialize on press of play
        hzChassis.initChassis();
        hzArm.initArmTeleOp();
        hzIntake.initIntakeTeleOp();

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            //hzGamepad1.runSubsystemByGamepadInput(hzChassis, hzArm, hzIntake);
            if (hzGamepad1.getLeftBumperPress()) {
                test_moveArm_blockLevelDown();
            }
            //If left bumper is pressed, move down a level
            if (hzGamepad1.getRightBumperPress()) {
                test_moveArm_blockLevelUp();
            }

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
        }
    }

    /**
     * Method to move arm up by a block level from current level in TeleOp and turn Brake Mode ON
     */
    public void test_moveArm_blockLevelUp(){
        hzArm.turnArmBrakeModeOn();
        currentLevel++;
        hzArm.armMotor.setTargetPosition(currentLevel*(-30));
        hzArm.runArmToLevel();
    }

    /**
     * Method to move arm down by a block level from current level in TeleOp
     * For blockLevel[1 to MAX_BLOCK_LEVEL], turn Brake Mode On
     * For blockLevel[0], set to groundlevel and turn Brake Mode Off
     */
    public void test_moveArm_blockLevelDown(){
        hzArm.turnArmBrakeModeOn();
        currentLevel--;
        hzArm.armMotor.setTargetPosition(currentLevel*(-30));
        hzArm.runArmToLevel();
        if (currentLevel == 0){
            hzArm.armMotor.setPower(0.0);
        }
    }


    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);
        telemetry.addData("backRightDrive.getCurrentPosition()", hzChassis.backRight.getCurrentPosition());
        telemetry.addData("backRightDrive.getCurrentPosition()", hzChassis.backLeft.getCurrentPosition());
        telemetry.addData("backRightDrive.getCurrentPosition()", hzChassis.frontRight.getCurrentPosition());
        telemetry.addData("backRightDrive.getCurrentPosition()", hzChassis.frontLeft.getCurrentPosition());
        //telemetry.addData("hzChassis.hook.getPosition : ", hzChassis.hook.getPosition());
        telemetry.addData("hzChassis.lefthook.getPosition : ", hzChassis.lefthook.getPosition());
        telemetry.addData("hzChassis.righthook.getPosition : ", hzChassis.righthook.getPosition());
        telemetry.addData("armMotor.isBusy : ", hzArm.armMotor.isBusy());
        telemetry.addData("armMotor.getTargetPosition : ", hzArm.armMotor.getTargetPosition());
        telemetry.addData("armMotor.getCurrentPosition : ", hzArm.armMotor.getCurrentPosition());
        telemetry.addData("armMotor.getMode : ", hzArm.armMotor.getMode());
        telemetry.addData("Arm.currentLevel : ", currentLevel);
        telemetry.addData("Arm.currentLevelPosition : ", (currentLevel*(-30)));
        telemetry.addData("Intake.left_grip.getPosition : ", hzIntake.left_grip.getPosition());
        telemetry.addData("Intake.right_grip.getPosition : ", hzIntake.right_grip.getPosition());
        telemetry.addData("Intake.wristCurrentPosition : ", hzIntake.wristCurrentPosition);
        telemetry.addData("Intake.wrist.getPosition : ", hzIntake.wrist.getPosition());

    }

}


