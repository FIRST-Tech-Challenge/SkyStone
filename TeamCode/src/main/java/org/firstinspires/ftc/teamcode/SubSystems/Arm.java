package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Definition of Robot Arm.
 * Arm has :
 *      1 motor to lift Intake to set levels
 *      Levels are :
 *          6 blockLevels for brick placement
 *          groundLevel as the arm is resting on ground
 *          detectSkystoneLevel as the arm is slightly lifted to have Skystone light sensor at
 *              height, used in autonomous mode
 *          aboveFoundationLevel as the arm is just above the foundation, used in autonomous
 *          onFoundationLevel as the arm is holding the foundation
 *      Arm Motor is 5201 Series, 53:1 Ratio, 105 RPM Spur Gear Motor w/Encoder
 *      Encoder Countable Events Per Revolution (Output Shaft)	1,497.325 (Rises & Falls of Ch A & B)
 *      Arm move 90degrees so max level is 1497.325/4 = 374 counts.
 *
 * @ArmMethods : moveArm_groundLevel()
 * @ArmMethods : moveArm_blockLevelUp()
 * @ArmMethods : moveArm_blockLevelDown()
 * @ArmMethods : moveArmToPlaceBlockAtLevel()
 * @ArmMethods : moveArmToLiftAfterBlockPlacement()
 * @ArmMethods : runArmToLevel()
 * @ArmAutoMethods : moveArm_detectSkystoneLevel()
 * @ArmAutoMethods : moveArm_aboveFoundationLevel(()
 * @ArmAutoMethods : moveArm_onFoundationLevel()
 *
 */
public class Arm {
    //Declare Arm Motor
    public DcMotor armMotor;

    //Declare Arm levels in arm motor encoder values set just above the block level
    //Arm Motor 5201 Series, 53:1 Ratio, 105 RPM Spur Gear Motor w/Encoder
    //Encoder Countable Events Per Revolution (Output Shaft)	1,497.325 (Rises & Falls of Ch A & B)
    //Arm move 90degrees so max level is 1497.325/4 = 374 counts. Add 30 counts for slippage compensation

    public int[] blockLevel = {
            +20, //ground level
            -130, //block level 1
            -220, //block level 2
            -310, //block level 3
            -400, //block level 4
            -490, //block level 5
            -580, //block level 6
            -630 //End Level 7
    };

    public int groundLevel = +20;
    public int detectSkystoneLevel = -220;//-70;
    public int aboveFoundationLevel = -130;
    public int onFoundationLevel = +30;
    int autoBlockPlacement = -150;
    int initLevel = -220;
    int initTeleOpLevel = +20;

    public int currentLevel = 0;
    int MAX_BLOCK_LEVEL = 7;

    //Timer for timing out Arm motion incase targetPosition cannot be achieved
    ElapsedTime ArmMotionTimeOut = new ElapsedTime();

    //Constructor
    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.dcMotor.get("arm");
    }

    /**
     * Initialize Arm - Reset, Set Zero Behavior to FLOAT (instead of BRAKE),
     * and mode to RUN_TO_POSITION (PID based rotation to
     * achieve the desire ed encoder count
     */
    public void initArm() {
        resetArm();
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveArm_initLevel();
    }

    /**
     * Initialize Arm for TeleOp- Reset, Set Zero Behavior to FLOAT (instead of BRAKE),
     * and mode to RUN_TO_POSITION (PID based rotation to
     * achieve the desire ed encoder count
     */
    public void initArmTeleOp() {
        resetArm();
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveArm_initTeleOpLevel();
    }

    /**
     * Method to set Arm brake mode to ON when Zero (0.0) power is applied.
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOn(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setPower(0.0);
    }

     /**
     * Method to set Arm brake mode to OFF when Zero (0.0) power is applied.
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnArmBrakeModeOff(){
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
     * Method to move Arm to groundlevel and turn Brake Mode OFF
     */
    public void moveArm_initLevel(){
        armMotor.setTargetPosition(initLevel);
        runArmToLevel();
    }

    /**
     * Method to move Arm to groundlevel and turn Brake Mode OFF
     */
    public void moveArm_initTeleOpLevel(){
        armMotor.setTargetPosition(initTeleOpLevel);
        runArmToLevel();
    }

    /**
     * Method to move Arm to groundlevel and turn Brake Mode OFF
     */
    public void moveArm_groundLevel(){
        armMotor.setTargetPosition(groundLevel);
        turnArmBrakeModeOff();
        runArmToLevel();
    }

    /**
     * Method to move Arm to detectSkystoneLevel and turn Brake Mode ON
     */
    public void moveArm_detectSkystoneLevel(){
        armMotor.setTargetPosition(detectSkystoneLevel);
        turnArmBrakeModeOn();
        runArmToLevel();
    }

    /**
     * Method to move Arm to aboveFoundationLevel and turn Brake Mode ON
     */
    public void moveArm_aboveFoundationLevel(){
        armMotor.setTargetPosition(aboveFoundationLevel);
        turnArmBrakeModeOn();
        runArmToLevel();
    }

    /**
     * Method to move Arm to onFoundationLevel and turn Brake Mode ON
     */
    public void moveArm_onFoundationLevel(){
        armMotor.setTargetPosition(onFoundationLevel);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        runArmToLevel();
    }

    /**
     * Method to move Arm to onFoundationLevel and turn Brake Mode ON
     */
    public void moveArm_AutoPlacementLevel(){
        armMotor.setTargetPosition(autoBlockPlacement);
        turnArmBrakeModeOn();
        runArmToLevel();
    }


    /**
     * Method to move arm up by a block level from current level in TeleOp and turn Brake Mode ON
     */
    public void moveArm_blockLevelUp(){
        turnArmBrakeModeOn();
        if (currentLevel < MAX_BLOCK_LEVEL) {
            armMotor.setTargetPosition(blockLevel[currentLevel+1]);
            currentLevel++;
            runArmToLevel();
        }
    }

    /**
     * Method to move arm down by a block level from current level in TeleOp
     * For blockLevel[1 to MAX_BLOCK_LEVEL], turn Brake Mode On
     * For blockLevel[0], set to groundlevel and turn Brake Mode Off
     */
    public void moveArm_blockLevelDown(){
        if (currentLevel > 1) {
            turnArmBrakeModeOn();
            armMotor.setTargetPosition(blockLevel[currentLevel-1]);
            currentLevel--;
            runArmToLevel();
        } else {
            turnArmBrakeModeOff();
            moveArm_groundLevel();
            currentLevel = 0;
            armMotor.setPower(0.0);
        }

    }

    /**
     * Method to move arm down a bit (DROP_BLOCK_HEIGHT) to place block on a level
     */
    public void moveArmToPlaceBlockAtLevel(){
        if (currentLevel >=1){
            turnArmBrakeModeOn();
            runArmToLevel();
        }
    }

    /**
     * Method to move arm up to currentLevel to release block on a level
     */
    public void moveArmToLiftAfterBlockPlacement(){
        if (currentLevel >=1) {
            turnArmBrakeModeOn();
            runArmToLevel();
        }
    }

    /**
     * Method to run motor to set to the set position
     */
    public void runArmToLevel() {

        ArmMotionTimeOut.reset();
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Turn Motors on
        armMotor.setPower(1.0);
    }

}
