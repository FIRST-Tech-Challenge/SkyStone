package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Definition of Robot Arm.
 * Arm has :
 *      1 motor to lift wrist and grip to set levels
 *      Levels are :
 *          6 blockLevels for brick placement
 *          groundLevel as the arm is resting on ground
 *          detectSkystoneLevel as the arm is slightly lifted to have Skystone light sensor at
 *              height, used in autonomous mode
 *          aboveFoundationLevel as the arm is just above the foundation, used in autonomous
 *          onFoundationLevel as the arm is holding the foundation
 *
 * @ArmMethods : moveArm_groundLevel()
 * @ArmMethods : moveArm_detectSkystoneLevel()
 * @ArmMethods : moveArm_aboveFoundationLevel(()
 * @ArmMethods : moveArm_onFoundationLevel()
 * @ArmMethods : moveArm_blockLevelUp()
 * @ArmMethods : moveArm_blockLevelDown()
 * @ArmMethods : moveArmToPlaceBlockAtLevel()
 * @ArmMethods : moveArmToLiftAfterBlockPlacement()
 * @ArmMethods : runArmToLevel()
 */

/**
 * Class Definition
 */
public class Arm {
    //Declare Arm Motor
    public DcMotor armMotor;

    //Declare Arm levels in arm motor encoder values set just above the block level
    int[] blockLevel = {
             0, //ground level
            -50, //block level 1
            -95, //block level 2
            -146, //block level 3
            -199, //block level 4
            -255, //block level 5
            -321 //block level 6
    };
    int groundLevel = 0;
    int detectSkystoneLevel = -20; //#TOBEFILLED correctly
    int aboveFoundationLevel = -50; //#TOBEFILLED correctly
    int onFoundationLevel = -10; //#TOBEFILLED correctly

    int currentLevel = 0;
    int MAXBLOCKLEVEL = 6;
    int DROP_BLOCK_HEIGHT = 10;
    int LIFT_BLOCK_HEIGHT = 40;

    //Constructor
    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.dcMotor.get("arm");
        initArm();
    }

    /**
     * Initialize Arm - Reset, Set Zero Behavior
     */
    public void initArm() {
        resetArm();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Reset function for motor encoders to be set to reset state of encoder.
     * Usage of this is typically followed by using setZeroBehaviour and then setting
     * the mode for the motor
     */
    public void resetArm() {

        DcMotor.RunMode runMode = armMotor.getMode();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(runMode);
    }

    /**
     * Function to set the behaviour of the motor on passing Zero power to the motor
     * @param zeroPowerBehavior could be BRAKE or FLOAT. When not defined, it is set
     *                          to UNKNOWN state, which is not desired.
     */
    public void setZeroBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        armMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Set the mode of the DC motor to RUN_WITHOUT_ENCODER (run at achievable velocity
     * RUN_USING_ENCODER (run at a targeted velocity or RUN_TO_POSITION (PID based rotation to
     * achieve the desited encoder count
     * @param runMode
     */
    public void setMode(DcMotor.RunMode runMode) {
        armMotor.setMode(runMode);
    }

    /**
     * Method to move Arm to groundlevel
     */
    public void moveArm_groundLevel(){
        armMotor.setTargetPosition(groundLevel);
        runArmToLevel();
    }

    /**
     * Method to move Arm to detectSkystoneLevel
     */
    public void moveArm_detectSkystoneLevel(){
        armMotor.setTargetPosition(detectSkystoneLevel);
        runArmToLevel();
    }

    /**
     * Method to move Arm to aboveFoundationLevel
     */
    public void moveArm_aboveFoundationLevel(){
        armMotor.setTargetPosition(aboveFoundationLevel);
        runArmToLevel();
    }

    /**
     * Method to move Arm to onFoundationLevel
     */
    public void moveArm_onFoundationLevel(){
        armMotor.setTargetPosition(onFoundationLevel);
        runArmToLevel();
    }

    /**
     * Method to move arm up by a block level from current level in TeleOp
     */
    public void moveArm_blockLevelUp(){
        if (currentLevel != MAXBLOCKLEVEL) {
            armMotor.setTargetPosition(blockLevel[currentLevel+1]);
            currentLevel++;
            runArmToLevel();
        }
    }

    /**
     * Method to move arm down by a block level from current level in TeleOp
     */
    public void moveArm_blockLevelDown(){
        if (currentLevel != 0) {
            armMotor.setTargetPosition(blockLevel[currentLevel-1]);
            currentLevel--;
            runArmToLevel();
        }
    }

    /**
     * Method to move arm down a bit to place block on a level
     */
    public void moveArmToPlaceBlockAtLevel(){
        int currentPosition = armMotor.getCurrentPosition();
        if (currentPosition <1) {
            armMotor.setTargetPosition(blockLevel[currentLevel] - DROP_BLOCK_HEIGHT);
        }
        runArmToLevel();
    }

    /**
     * Method to move arm up a bit to release block on a level
     */
    public void moveArmToLiftAfterBlockPlacement(){
        int currentPosition = armMotor.getCurrentPosition();
        if (currentPosition <1) {
            armMotor.setTargetPosition(blockLevel[currentLevel] + LIFT_BLOCK_HEIGHT);
        }
        runArmToLevel();
    }

    /**
     * Method to run motor to set to the set position
     */

    public void runArmToLevel() {
        //armMotor.setTargetPosition(blockLevel[level]);;
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        /*if(level!=0){
            armMotor.setPower(1);
        }else{
            armMotor.setPower(0);
        }*/
    }

    public void setArm(int level) {
        armMotor.setTargetPosition(blockLevel[level]-10);
    }


    public void setArmCheck(double error) {

    }

}
