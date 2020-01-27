package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

public class RobotHardware {
    ExpansionHubMotor rrMotor, rlMotor, frMotor, flMotor, intakeMotorLeft, intakeMotorRight, liftMotor, sliderMotor;
    ExpansionHubEx expansionHub1, expansionHub2;
    RevBulkData bulkData1, bulkData2;
    ExpansionHubServo clampRotationServo, blockHolderServo1, blockHolderServo2, leftFoundationServo, rightFoundationServo,
            capStoneServo;

    Servo conveyor1, conveyor2, tapeDrive;
    DigitalChannel sliderTouchChannel;
    Rev2mDistanceSensor rightDistanceSensor;
    //AnalogInput rightDistanceSensor, leftBackDistanceSensor, rightBackDistanceSensor;
    RobotProfile profile;
    boolean isPrototype = false;
    BNO055IMU imu1, imu2;

    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        this.profile = profile;
        try {
            if (hardwareMap.get("LiftMotor")!=null) {
                isPrototype = false;
            }
        }
        catch (IllegalArgumentException ex) {
            isPrototype = true;
        }
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        rrMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("RRMotor");
        rlMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("RLMotor");
        frMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("FRMotor");
        flMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("FLMotor");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
        imu1.initialize(parameters);
        capStoneServo = (ExpansionHubServo) hardwareMap.servo.get("CapStoneServo");

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
        if (!isPrototype) {
            expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
            intakeMotorLeft = (ExpansionHubMotor) hardwareMap.dcMotor.get("LeftIntakeMotor");
            intakeMotorRight = (ExpansionHubMotor) hardwareMap.dcMotor.get("RightIntakeMotor");
            liftMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("LiftMotor");
            sliderMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("LiftSliderMotor");
            clampRotationServo = (ExpansionHubServo) hardwareMap.servo.get("ClampRotationServo");
            blockHolderServo1 = (ExpansionHubServo) hardwareMap.servo.get("BlockHolderServo1");
            blockHolderServo2 = (ExpansionHubServo) hardwareMap.servo.get("BlockHolderServo2");
            leftFoundationServo = (ExpansionHubServo) hardwareMap.servo.get("LeftFoundationServo");
            rightFoundationServo = (ExpansionHubServo) hardwareMap.servo.get("RightFoundationServo");
            conveyor1 = hardwareMap.servo.get("Conveyor1");
            conveyor2 = hardwareMap.servo.get("Conveyor2");
            tapeDrive = hardwareMap.servo.get("TapeDrive");
            rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "RightDistanceSensor");
            //leftBackDistanceSensor = hardwareMap.analogInput.get("LeftBackDistanceSensor");
            //rightBackDistanceSensor = hardwareMap.analogInput.get("RightBackDistanceSensor");
            // We use BulkData read, so can't use touch sensor object
            //slideTouchSensor = hardwareMap.touchSensor.get("SlideTouchSensor");
            sliderTouchChannel = hardwareMap.digitalChannel.get("SlideTouchSensor");
            sliderTouchChannel.setMode(DigitalChannel.Mode.INPUT);

            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftMotor.setTargetPosition(0);
            sliderMotor.setTargetPosition(0);
        }
    }

    public void getBulkData1() {
        bulkData1 = expansionHub1.getBulkInputData();
    }

    public void getBulkData2() {
        if (!isPrototype) {
            bulkData2 = expansionHub2.getBulkInputData();
        }
        else {
            // so we have something
            bulkData2 = expansionHub1.getBulkInputData();
        }
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
            return bulkData2.getMotorCurrentPosition(liftMotor);
        }
        else if(encoder == EncoderType.SLIDER) {
            return bulkData2.getMotorCurrentPosition(sliderMotor);
        } else {
            return 0;
        }
    }

    public void mecanumDriveTest(double power, double angle, double rotation, int sign){
        double frontLeft = 0;
        double frontRight = 0;
        double rearLeft = 0;
        double rearRight = 0;
        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        //double robotAngle = Math.PI / 2 - angle - Math.PI / 4;
        if(sign == 0) {
            frontLeft = power * Math.cos(angle) + rotation;
            frontRight = power * Math.sin(angle) - rotation;
            rearLeft = power * Math.sin(angle) + rotation;
            rearRight = power * Math.cos(angle) - rotation;
        }else if(sign ==1){  //left side less encoder counts
            frontLeft = power *0.88 * Math.cos(angle) + rotation;
            frontRight = power * Math.sin(angle) - rotation;
            rearLeft = power * 0.88 * Math.sin(angle) + rotation;
            rearRight = power * Math.cos(angle) - rotation;
        }else if(sign == 2){   //right side less encoder counts
            frontLeft = power * Math.cos(angle) + rotation;
            frontRight = power * 0.88 * Math.sin(angle) - rotation;
            rearLeft = power * Math.sin(angle) + rotation;
            rearRight = power * 0.88 * Math.cos(angle) - rotation;
        }

        double biggest = 0.1;
        if (Math.abs(frontRight) > biggest){
            biggest = Math.abs(frontRight);
        }
        if (Math.abs(rearLeft) > biggest){
            biggest = Math.abs(rearLeft);
        }
        if (Math.abs(rearRight) > biggest){
            biggest = Math.abs(rearRight);
        }
        if (Math.abs(frontLeft) > biggest){
            biggest = Math.abs(frontLeft);
        }

        power = (power == 0 && rotation !=0) ? 1 : power;
        frontLeft = frontLeft/biggest*power;
        frontRight = frontRight/biggest*power;
        rearLeft = rearLeft/biggest*power;
        rearRight = rearRight/biggest*power;

        setMotorPower(frontLeft, frontRight, rearLeft, rearRight);
    }

    public void mecanumDrive2(double power, double angle, double rotation){

        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        double robotAngle = Math.PI / 2 - angle - Math.PI / 4;
        double frontLeft = power * Math.cos(robotAngle) + rotation;
        double frontRight = power * Math.sin(robotAngle) - rotation;
        double rearLeft = power * Math.sin(robotAngle) + rotation;
        double rearRight = power * Math.cos(robotAngle) - rotation;


        double biggest = 0;
        if (Math.abs(frontRight) > biggest){
            biggest = Math.abs(frontRight);
        }
        if (Math.abs(rearLeft) > biggest){
            biggest = Math.abs(rearLeft);
        }
        if (Math.abs(rearRight) > biggest){
            biggest = Math.abs(rearRight);
        }
        if (Math.abs(frontLeft) > biggest){
            biggest = Math.abs(frontLeft);
        }

        power = Math.max(power, Math.abs(rotation));
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

//    public void rotateGrabberOriginPos() {
//        clampRotationServo.setPosition(profile.hardwareSpec.clampAngleNormal);
//    }

    public void rotateGrabberOriginPos() {
        clampRotationServo.setPosition(profile.hardwareSpec.clampAngleNormal);
    }

    public void rotateGrabberOutPos() {
        clampRotationServo.setPosition(profile.hardwareSpec.clampAngleSide);
    }

    public void setLiftPosition(int liftPosition){
        setLiftPosition(liftPosition, 0.8);
    }

    public void setLiftPosition(int liftPosition, double power){
//        long currPos = this.getEncoderCounts(EncoderType.LIFT);
//        if (liftPosition>currPos) {
//            // going up, then can not higher than 3000
//            liftMotor.setTargetPosition(Math.min(liftPosition, 3000));
//        }
//        else {
//            // going down, then can not go lower than 0
////            liftMotor.setTargetPosition(Math.max(0, liftPosition));
//        }
//        // Make sure the lift position >0 and < 4000 (around 11 bricks)
        liftMotor.setTargetPosition(liftPosition);
        liftMotor.setPower(power);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetSlideToZero() {
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setSliderPosition(int sliderPosition) {
        sliderMotor.setTargetPosition(sliderPosition);
        //sliderMotor.setPower(0.5);

        //12/12, increase slide speed, so lift can come down quickly
        sliderMotor.setPower(0.8);

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
        else if (clampPosition == ClampPosition.INITIAL) {
            blockHolderServo1.setPosition(profile.hardwareSpec.clampS1Init);
            blockHolderServo2.setPosition(profile.hardwareSpec.clampS2Init);
        }
        else if (clampPosition == ClampPosition.RELEASE_1) {
            blockHolderServo1.setPosition(profile.hardwareSpec.clampS1WheelRelease);
            blockHolderServo2.setPosition(profile.hardwareSpec.clampS2Init);
        }
        else if (clampPosition == ClampPosition.RELEASE_2) {
            blockHolderServo1.setPosition(profile.hardwareSpec.clampS2WheelRelease);
            blockHolderServo2.setPosition(profile.hardwareSpec.clampS1Init);
        } else if (clampPosition == ClampPosition.OPEN_SIDEWAYS) {
            blockHolderServo1.setPosition(profile.hardwareSpec.clampS1Close);
            blockHolderServo2.setPosition(profile.hardwareSpec.clampS2OpenLarge);
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

    public void startIntakeWheels() {
        intakeMotorLeft.setPower(0.5);
        intakeMotorRight.setPower(-0.5);
        conveyor1.setPosition(0.2);
        conveyor2.setPosition(0.2);
    }

    public void stopIntakeWheels() {
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);
        conveyor1.setPosition(0.5);
        conveyor2.setPosition(0.5);
    }

    public void reverseIntakeWheels(){
        intakeMotorLeft.setPower(-0.5);
        intakeMotorRight.setPower(0.5);
        conveyor1.setPosition(0.8);
        conveyor2.setPosition(0.8);
    }

    public void setIntakeDirection(IntakeDirection direction){
        if(direction == IntakeDirection.TAKE_IN){
            startIntakeWheels();
        } else if(direction == IntakeDirection.STOP){
            stopIntakeWheels();
        } else{
            reverseIntakeWheels();
        }
    }


    public void setCapStoneServo(CapPosition capPosition){
        if(capPosition == CapPosition.CAP_UP){
            capStoneServo.setPosition(profile.hardwareSpec.capStoneUp);
        }
        else if(capPosition == CapPosition.CAP_DOWN){
            capStoneServo.setPosition(profile.hardwareSpec.capStoneDown);
        }
        else if(capPosition == CapPosition.CAP_OTHER){
            capStoneServo.setPosition(profile.hardwareSpec.capStoneOther);
        }
    }

    public boolean sliderTouched() {
        // digital channel: low - touched, high - not touch
        return !bulkData2.getDigitalInputState(sliderTouchChannel);
    }

    public void setClampAnglePosition(ClampAnglePosition clampAnglePosition){
        if(clampAnglePosition ==ClampAnglePosition.NORMAL)
            clampRotationServo.setPosition(profile.hardwareSpec.clampAngleNormal);
        else if(clampAnglePosition == ClampAnglePosition.SIDE)
            clampRotationServo.setPosition(profile.hardwareSpec.clampAngleSide);
        else if(clampAnglePosition == ClampAnglePosition.BACK)
            clampRotationServo.setPosition(profile.hardwareSpec.clampAngleBack);
    }

    public void extendTape() {
        tapeDrive.setPosition(0.2);
    }

    public void retractTake() {
        tapeDrive.setPosition(0.8);
    }

    public void stopTape() {
        tapeDrive.setPosition(0.5);
    }

    /**
     * WARNING WARNING WARNING ****
     * Sensor distance call not supported by BulkData read, this call takes 33ms to complete
     * do not call as part of the loop constantly.
     * @return
     */
    public double getRightDistance() {
        return rightDistanceSensor.getDistance(DistanceUnit.CM);
    }


    public enum EncoderType {LEFT, RIGHT, HORIZONTAL, LIFT, SLIDER}
    public enum ClampAnglePosition{NORMAL, SIDE, BACK}

    public enum ClampPosition {OPEN, CLOSE, INITIAL, RELEASE_1, RELEASE_2, OPEN_SIDEWAYS}
    public enum HookPosition {HOOK_ON, HOOK_OFF}
    public enum CapPosition {CAP_UP, CAP_DOWN, CAP_OTHER}
    public enum IntakeDirection {TAKE_IN, RELEASE, STOP}

}
