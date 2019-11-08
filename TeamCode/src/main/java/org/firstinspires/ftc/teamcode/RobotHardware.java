package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

public class RobotHardware {
//    final double DEFAULT_INTAKE_ROTATION_SERVO = 0.86;
//    final double STRAIGHT_INTAKE_ROTATION_SERVO = 0.53;
//    final double OPEN_BLOCK_HOLDER_SERVO_S1 = 0.695;
//    final double CLOSE_BLOCK_HOLDER_SERVO_S1 = 0.595;
//    final double OPEN_BLOCK_HOLDER_SERVO_S2 = 0.40;
//    final double CLOSE_BLOCK_HOLDER_SERVO_S2 = 0.50;
//    final double PARK_FOUNDATION_SERVO_S0 = 0.50;
//    final double LOCK_FOUNDATION_SERVO_S0 = 0.83;
//    final double PARK_FOUNDATION_SERVO_S1 = 0.50;
//    final double LOCK_FOUNDATION_SERVO_S1 = 0.18;

    ExpansionHubMotor rrMotor, rlMotor, frMotor, flMotor, intakeMotorLeft, intakeMotorRight, liftMotor, sliderMotor;
    ExpansionHubEx expansionHub1, expansionHub2;
    RevBulkData bulkData1, bulkData2;
    ExpansionHubServo clampRotationServo, blockHolderServo1, blockHolderServo2, leftFoundationServo, rightFoundationServo;
    RobotProfile profile;
    BNO055IMU imu1, imu2;

    public enum EncoderType {LEFT, RIGHT, HORIZONTAL, LIFT, SLIDER}
    public enum ClampPosition {OPEN, CLOSE}
    public enum HookPosition {HOOK_ON, HOOK_OFF}

    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        this.profile = profile;
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        rrMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("RRMotor");
        rlMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("RLMotor");
        frMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("FRMotor");
        flMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("FLMotor");
        intakeMotorLeft = (ExpansionHubMotor) hardwareMap.dcMotor.get("LeftIntakeMotor");
        intakeMotorRight = (ExpansionHubMotor) hardwareMap.dcMotor.get("RightIntakeMotor");
        liftMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("LiftMotor");
        sliderMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("LiftSliderMotor");
        clampRotationServo = (ExpansionHubServo) hardwareMap.servo.get("ClampRotationServo");
        blockHolderServo1 = (ExpansionHubServo) hardwareMap.servo.get("BlockHolderServo1");
        blockHolderServo2 = (ExpansionHubServo) hardwareMap.servo.get("BlockHolderServo2");
        leftFoundationServo = (ExpansionHubServo) hardwareMap.servo.get("LeftFoundationServo");
        rightFoundationServo = (ExpansionHubServo) hardwareMap.servo.get("RightFoundationServo");
        //imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
        //imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
       // imu1 = (BNO055IMU) hardwareMap.("imu1");
       // imu2 =(BNO055IMU) hardwareMap.i2cDevice.get("imu2");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
        imu1.initialize(parameters);


        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setTargetPosition(0);
        sliderMotor.setTargetPosition(0);


        Logger.logFile("lfMotor = "+ flMotor);
        Logger.logFile("frMotor = "+ frMotor);
        Logger.logFile("rlMotor = "+ rlMotor);
        Logger.logFile("rrMotor = "+ rrMotor);
        Logger.logFile("intake motorR=" + intakeMotorRight);
        Logger.logFile("intake motorL=" + intakeMotorLeft);
    }

    public void getBulkData1() {
        bulkData1 = expansionHub1.getBulkInputData();
    }

    public void getBulkData2() {
        bulkData2 = expansionHub2.getBulkInputData();
    }

    public int getEncoderCounts(EncoderType encoder) {
        if(encoder == EncoderType.LEFT) {
            return profile.hardwareSpec.leftEncodeForwardSign * bulkData1.getMotorCurrentPosition(rlMotor);
        }
        else if(encoder == EncoderType.RIGHT) {
            return profile.hardwareSpec.rightEncoderForwardSign * bulkData1.getMotorCurrentPosition(rrMotor);
        }
        else if(encoder == EncoderType.HORIZONTAL) {
            return profile.hardwareSpec.horizontalEncoderForwardSign * bulkData1.getMotorCurrentPosition(frMotor);
        }
        else if(encoder == EncoderType.LIFT) {
            return liftMotor.getCurrentPosition();
            //return bulkData2.getMotorCurrentPosition(liftMotor);
        }
        else if(encoder == EncoderType.SLIDER) {
             return sliderMotor.getCurrentPosition();
            //return bulkData2bulkData2.getMotorCurrentPosition(liftMotor);
        } else {
            return 0;
        }
    }

    public void mecanumDrive(double power, double angle, double rotation){

        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        //double robotAngle = Math.PI / 2 - angle - Math.PI / 4;
        double frontLeft = power * Math.cos(angle) + rotation;
        double frontRight = power * Math.sin(angle) - rotation;
        double rearLeft = power * Math.sin(angle) + rotation;
        double rearRight = power * Math.cos(angle) - rotation;


        double biggest = 1;
        if (Math.abs(frontRight) > biggest){
            biggest = Math.abs(frontRight);
        } else if (Math.abs(rearLeft) > biggest){
            biggest = Math.abs(rearLeft);
        } else if (Math.abs(rearRight) > biggest){
            biggest = Math.abs(rearRight);
        } else if (Math.abs(frontLeft) > biggest){
            biggest = Math.abs(frontLeft);
        }

        power = (power == 0 && rotation !=0) ? 1 : power;
        frontLeft = frontLeft/biggest*power;
        frontRight = frontRight/biggest*power;
        rearLeft = rearLeft/biggest*power;
        rearRight = rearRight/biggest*power;

        setMotorPower(frontLeft, frontRight, rearLeft, rearRight);
    }

    public void setMotorPower(double flPower, double frPower, double rlPower, double rrPower) {
        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        rlMotor.setPower(rlPower);
        rrMotor.setPower(rrPower);
    }

    public void setMotorStopBrake(boolean brake) {
        flMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        frMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rlMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rrMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //10/14/2019 Reily refactor field oriented to new method
    public void switchToFieldOrientated(double forward, double strafe, double heading) {
        //double power = Math.hypot(forward,strafe);
        //double angle = Math.atan2(forward,strafe)- Math.PI / 4;

        double temp = forward * Math.cos(heading) + strafe * Math.sin(heading);
        strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;
    }

    public void rotateGrabberForPickup() {
        clampRotationServo.setPosition(profile.hardwareSpec.clampAngleReady);
    }

    public void rotateGrabberOriginPos() {
        clampRotationServo.setPosition(profile.hardwareSpec.clampAngleInit);
    }

    public void setLiftPosition(int liftPosition){
        // Make sure the lift position >0 and < 4000 (around 11 bricks)
        liftMotor.setTargetPosition(Math.max(0, Math.min(liftPosition, 1730)));
        liftMotor.setPower(0.8);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setSliderPosition(int sliderPosition) {
        sliderMotor.setTargetPosition(sliderPosition);
        sliderMotor.setPower(0.3);
        sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setClampPosition(ClampPosition clampPosition){
        if(clampPosition == ClampPosition.OPEN){
            blockHolderServo1.setPosition(profile.hardwareSpec.clampS1Open);
            blockHolderServo2.setPosition(profile.hardwareSpec.clampS2Open);
        }
        else if(clampPosition == ClampPosition.CLOSE){
            blockHolderServo1.setPosition(profile.hardwareSpec.clampS1Close);
            blockHolderServo2.setPosition(profile.hardwareSpec.clampS2Close);
        }
    }

    public void setHookPosition(HookPosition hookPosition){
        if(hookPosition == HookPosition.HOOK_OFF){
            leftFoundationServo.setPosition(profile.hardwareSpec.hookS0Open);
            rightFoundationServo.setPosition(profile.hardwareSpec.hookS1Open);
        }else if(hookPosition == HookPosition.HOOK_ON){
            leftFoundationServo.setPosition(profile.hardwareSpec.hookS0Close);
            rightFoundationServo.setPosition(profile.hardwareSpec.hookS1Close);
        }
    }

    public float getGyroAngle() {
        float angle1 = -imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //float angle2 = robotHardware.imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        //return (angle1 + angle2) / 2;
        return angle1;
    }

    public void startIntakeWheel() {
        intakeMotorLeft.setPower(0.3);
        intakeMotorRight.setPower(-0.3);
    }

    public void stopIntakeWheel() {
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);
    }
    public void reverseIntakeWheels(){
        intakeMotorLeft.setPower(-0.3);
        intakeMotorRight.setPower(0.3);
    }

}
