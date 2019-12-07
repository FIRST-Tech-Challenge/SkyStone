package org.firstinspires.ftc.teamcode.autoRes.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Chassis;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Autonomous Mode Usecase 2
 * Description : Start at Blue side, Grip Foundation with hook, and position it into Building Site.
 *
 * Steps:
 * Robot starts between A4, A5 such that it can slight in front of skybridge neutral zone floor
 * On start, robot opens wrist to front position
 * Lift Arm to AboveFoundation level
 * Move robot to in between C5 and C6
 * Move forward till Chassis bumber limit swich is pressed.
 * Drop Arm to OnFoundation level
 * Move Robot Left toward A4 (for XX rotations). Friction will cause Robot to rotate towards A6
 * Pull back till wall is hit (Motor does not move)
 * Slide left till Motor does not move (Foundation corner on Edge)
 * Push forward to move foundation to end of line
 * Lift Arm to Above foundation level
 * Move back till wall is hit
 * Move right by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge
 */

@Autonomous(name = "HzAutoRunDistance", group = "Autonomous")
public class AutonomousUC2 extends LinearOpMode {

    public Intake autoIntake;
    public Arm autoArm;
    public Chassis autoChassis;

    public int skystonePosition;

    ElapsedTime AutonomousTimeOut = new ElapsedTime();

    /**
     * Template runOpMode code. Only change Usecase function and call here.
     * Refer to Autonomous Command Syntax to put right values
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        autoIntake = new Intake(hardwareMap);
        autoArm = new Arm(hardwareMap);
        autoChassis = new Chassis(hardwareMap);

        //Robot starts on A2
        waitForStart();
        while (opModeIsActive()) {
            AutonomousUC2Commands();
        }
    }

    public void AutonomousUC2Commands(){

        //Robot starts between A4, A5 such that it can slight in front of skybridge neutral zone floor

        //On start, Lift arm and robot opens wrist to front position
        //initArm() and initIntake() should do this on class initialization

        //Lift Arm to AboveFoundation level
        autoArm.moveArm_aboveFoundationLevel();

        //Move robot to in between C5 and C6

        //Move forward till Chassis bumber limit swich is pressed.
        //Drop Arm to OnFoundation level
        //Move Robot Left toward A4 (for XX rotations). Friction will cause Robot to rotate towards A6
        //Pull back till wall is hit (Motor does not move)
        //Slide left till Motor does not move (Foundation corner on Edge)
        //Push forward to move foundation to end of line
        //Lift Arm to Above foundation level
        //Move back till wall is hit
        //Move right by distance or till Chassis light sensor does not detect Blue line to be under blue skybridge




    }

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
