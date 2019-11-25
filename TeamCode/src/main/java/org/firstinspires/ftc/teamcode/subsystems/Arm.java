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
 * @ArmMethods : setArm_groundLevel()
 * @ArmMethods : setArm_detectSkystoneLevel()
 * @ArmMethods : setArm_aboveFoundationLevel(()
 * @ArmMethods : setArm_onFoundationLevel()
 * @ArmMethods : setArm_blockLevelUp()
 * @ArmMethods : setArm_blockLevelDown()
 * @ArmMethods : moveArmToPlaceBlockAtLevel()
 * @ArmMethods : moveArmToLiftAfterBlockPlacement()
 */

/**
 * Class Definition
 */
public class Arm {
    //Declare Arm Motor
    public DcMotor arm;

    //Declare Arm levels in arm motor encoder values set just above the block level
    int[] blockLevel = {
             0, //ground level
            -40, //block level 1
            -85, //block level 2
            -136, //block level 3
            -189, //block level 4
            -245, //block level 5
            -311 //block level 6
    };

    //Constructor
    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.dcMotor.get("arm");
        initArm();
    }

    public DcMotor getArm() {
        return arm;
    }

    public void initArm() {
        reset();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArm(int level) {
        arm.setTargetPosition(levelAngles[level]-10);
    }

    public void setArmCheck(double error) {

    }

    public void run(int level) {
        setArm(level);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(level!=0){
            arm.setPower(1);
        }else{
            arm.setPower(0);
        }
    }
}
