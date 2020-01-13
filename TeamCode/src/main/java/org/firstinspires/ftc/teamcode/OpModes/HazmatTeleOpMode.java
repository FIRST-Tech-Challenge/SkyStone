package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.*;

/**
 * TeleOpMode for Team Hazmat<BR>
 * Includes autoplacement routince for automatically placing block on tower
 */
@TeleOp(name = "HazmatTeleOpMode", group = "Teleop")
public class HazmatTeleOpMode extends LinearOpMode{

    public boolean HzDEBUG_FLAG = true;

    int hzRobotNum; //Set to 1 for main robot, 2 for second robot

    HzGamepad1 hzGamepad1;
    Chassis hzChassis;
    Arm hzArm;
    Intake hzIntake;


    @Override
    public void runOpMode() {

        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        hzChassis = new Chassis(hardwareMap);
        hzArm = new Arm(hardwareMap);
        hzIntake = new Intake(hardwareMap);
        hzGamepad1 = new HzGamepad1(gamepad1);

        telemetry.addData("Hazmat TeleOp Mode", "v:1.0");

        //Wait for pressing plan on controller
        waitForStart();

        //Initialize on press of play
        hzChassis.initChassis();
        hzArm.initArmTeleOp();
        hzIntake.initIntakeTeleOp();

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            //Run per Gamepad input
            hzGamepad1.runSubsystemByGamepadInput(hzChassis, hzArm, hzIntake);

            //Activate auto placement
            autoPlaceBlock();

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
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
        telemetry.addData("hzGamepad1.getLeftTrigger()",hzGamepad1.getLeftTrigger());
        telemetry.addData("hzChassis.hook.getPosition : ", hzChassis.hook.getPosition());
        telemetry.addData("armMotor.isBusy : ", hzArm.armMotor.isBusy());
        telemetry.addData("armMotor.getTargetPosition : ", hzArm.armMotor.getTargetPosition());
        telemetry.addData("armMotor.getCurrentPosition : ", hzArm.armMotor.getCurrentPosition());
        telemetry.addData("armMotor.getMode : ", hzArm.armMotor.getMode());
        telemetry.addData("Arm.currentLevel : ", hzArm.currentLevel);
        telemetry.addData("Arm.currentLevelPosition : ", -(hzArm.currentLevel)*50);
        telemetry.addData("Intake.left_grip.getPosition : ", hzIntake.left_grip.getPosition());
        telemetry.addData("Intake.right_grip.getPosition : ", hzIntake.right_grip.getPosition());
        telemetry.addData("Intake.wristCurrentPosition : ", hzIntake.wristCurrentPosition);
        telemetry.addData("Intake.wrist.getPosition : ", hzIntake.wrist.getPosition());

    }

    /**
     * Method for automatic placement of block to the brock tower<BR>
     *
     * Initial state :<BR>
     *  - Block is gripped and held at the level to place.<BR>
     *  - Robot arm is aligned such that block to be placed is right above the existing tower<BR>
     * Activation :<BR>
     *  - Automation button combo is pressed (LeftTrigger and Right Trigger are both fully pressed)<BR>
     *  - With the setup state, Move robot forward and touch base (frontLeftTouchSensor is pressed)<BR>
     * Motion :<BR>
     *  - While automation button combo is pressed<BR>
     *  - Robot moves back distance such that block is right above the tower<BR>
     *  - Arm is lowered such that block is on the tower<BR>
     *  - Grip is opened<BR>
     *  - Robot moves back to release arm and grip from on top of tower<BR>
     *<BR>
     *  Automatic works only when the combo button is kept pressed simultaneously and stops when released<BR>
     */
    public void autoPlaceBlock(){

        //Distance to be moved for each block level
        int[] blockLevelDistance = {
                -8, //ground level
                -7, //block level 1
                -6, //block level 2
                -4, //block level 3
                -2, //block level 4
                -1, //block level 5
                0, //block level 6
                0 //End Level 7
        };

        // While automation button combo is pressed (LeftTrigger and Right Trigger are both fully pressed)
        // Robot moves back distance such that block is right above the tower
        if(hzGamepad1.getRightTrigger()==1 && hzGamepad1.getLeftTrigger()==1
                && hzChassis.frontleftChassisTouchSensorIsPressed()){
            hzChassis.runFwdBackLeftRight(blockLevelDistance[hzArm.currentLevel],0, 0.1, this);
        }

        //Arm is lowered such that block is on the tower
        if(hzGamepad1.getRightTrigger()==1 && hzGamepad1.getLeftTrigger()==1){
            hzArm.moveArm_blockLevelDown();
        }

        //Grip is opened
        if(hzGamepad1.getRightTrigger()==1 && hzGamepad1.getLeftTrigger()==1){
            hzIntake.openGrip();
        }

        //Robot moves back to release arm and grip from on top of tower
        if(hzGamepad1.getRightTrigger()==1 && hzGamepad1.getLeftTrigger()==1){
            hzChassis.runFwdBackLeftRight(-4,0, 0.1, this);
        }
    }

}


