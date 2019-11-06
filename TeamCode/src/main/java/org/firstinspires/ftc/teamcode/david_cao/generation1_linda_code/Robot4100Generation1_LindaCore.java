package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.chassiscontrollers.OmniDrive;
import org.darbots.darbotsftclib.libcore.common_robotcores.OmniChassisRobotCore;
import org.darbots.darbotsftclib.libcore.sensors.gyros.BNO055Gyro;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.sensors.servos.TimeControlledServo;
import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class Robot4100Generation1_LindaCore extends OmniChassisRobotCore {
    public enum IntakeSystemStatus{
        SUCK,
        VOMIT,
        STOP
    }
    private Servo m_DragServoL, m_DragServoR;
    private Servo m_Grabber;
    private TimeControlledServo m_GrabberRot;
    private Servo m_StoneOrientServo;
    private RobotServoUsingMotor m_linearSlide;
    private DcMotor m_IntakeLeft, m_IntakeRight;
    private Servo m_AutoDragStoneServoLeft, m_AutoDragStoneServoRight;
    private Servo m_CapStoneServo;

    public Robot4100Generation1_LindaCore(HardwareMap hardwares) {
        super("4100Generation1_LindaCore.log",hardwares, Robot4100Generation1_Settings.wheelPosition,Robot4100Generation1_Settings.wheelRadius,"LF","LB","RF","RB",Robot4100Generation1_Settings.motorType,Robot4100Generation1_Settings.CHASSIS_TIMEOUTENABLE,Robot4100Generation1_Settings.CHASSIS_TIMEOUTFACTOR);
        this.getChassis().setLinearMotionDistanceFactor(0.678);
        this.getChassis().setRotationalMotionDistanceFactor(1.4);

        this.m_DragServoL = hardwares.servo.get("servoDragLeft");
        this.m_DragServoR = hardwares.servo.get("servoDragRight");
        this.m_Grabber = hardwares.servo.get("servoGrabber");

        Servo GrabberRotServo = hardwares.servo.get("servoGrabberRot");
        this.m_GrabberRot = new TimeControlledServo(GrabberRotServo,Robot4100Generation1_Settings.GRABBERROTSERVO_TYPE,Robot4100Generation1_Settings.GRABBERROTSERVO_INSIDEPOS,true);


        RobotMotor LinearSlideMotor = new RobotMotorWithEncoder(hardwares.dcMotor.get("motorLinearSlide"),Robot4100Generation1_Settings.linearSlideMotorType);
        LinearSlideMotor.setDirectionReversed(true);
        RobotMotorController linearSlideController = new RobotMotorController(LinearSlideMotor,Robot4100Generation1_Settings.LINEARSLIDE_TIMEOUTCONTROLENABLE,Robot4100Generation1_Settings.LINEARSLIDE_TIMEOUTFACTOR);
        this.m_linearSlide = new RobotServoUsingMotor(linearSlideController,Robot4100Generation1_Settings.LINEARSLIDE_START,Robot4100Generation1_Settings.LINEARSLIDE_MIN,Robot4100Generation1_Settings.LINEARSLIDE_MAX);

        this.m_IntakeLeft = hardwares.dcMotor.get("motorIntakeLeft");
        this.m_IntakeRight = hardwares.dcMotor.get("motorIntakeRight");

        this.m_StoneOrientServo = hardwares.servo.get("servoStoneOrient");

        this.m_AutoDragStoneServoLeft = hardwares.servo.get("servoAutoDragStoneLeft");
        this.m_AutoDragStoneServoRight = hardwares.servo.get("servoAutoDragStoneRight");

        this.m_CapStoneServo = hardwares.servo.get("servoCapStone");

        this.setCapStoneServoToDeposit(false);
        this.setOrientServoToOrient(false);
        this.setDragServoToDrag(false);
        this.setAutonomousDragStoneServoLeftToDrag(false);
        this.setAutonomousDragStoneServoRightToDrag(false);
    }


    public void setAutonomousDragStoneServoLeftToDrag(boolean toDrag){
        if(toDrag){
            this.m_AutoDragStoneServoLeft.setPosition(Robot4100Generation1_Settings.AUTONOMOUSDRAGSTONESERVO_LEFT_OUTPOS);
        }else{
            this.m_AutoDragStoneServoLeft.setPosition(Robot4100Generation1_Settings.AUTONOMOUSDRAGSTONESERVO_LEFT_INPOS);
        }
    }
    public void setAutonomousDragStoneServoRightToDrag(boolean toDrag){
        if(toDrag){
            this.m_AutoDragStoneServoRight.setPosition(Robot4100Generation1_Settings.AUTONOMOUSDRAGSTONESERVO_RIGHT_OUTPOS);
        }else{
            this.m_AutoDragStoneServoRight.setPosition(Robot4100Generation1_Settings.AUTONOMOUSDRAGSTONESERVO_RIGHT_IN);
        }
    }

    public void setCapStoneServoToDeposit(boolean toDeposit){
        if(toDeposit){
            this.m_CapStoneServo.setPosition(Robot4100Generation1_Settings.CAPSTONESERVO_DEPOSITPOS);
        }else{
            this.m_CapStoneServo.setPosition(Robot4100Generation1_Settings.CAPSTONESERVO_INITIALPOS);
        }
    }

    public RobotServoUsingMotor getLinearSlide(){
        return this.m_linearSlide;
    }

    public void setLinearSlideToRecieveStonePos(double speed){
        this.getLinearSlide().replaceTask(new TargetPosTask(null,Robot4100Generation1_Settings.LINEARSLIDE_GRAB,speed));
    }
    public void setDragServoToDrag(boolean drag){
        if(drag){
            this.m_DragServoL.setPosition(Robot4100Generation1_Settings.DRAGSERVO_DRAGPOS_L);
            this.m_DragServoR.setPosition(Robot4100Generation1_Settings.DRAGSERVO_DRAGPOS_R);
        }else{
            this.m_DragServoL.setPosition(Robot4100Generation1_Settings.DRAGSERVO_RESTPOS_L);
            this.m_DragServoR.setPosition(Robot4100Generation1_Settings.DRAGSERVO_RESTPOS_R);
        }
    }

    public void setGrabberServoToGrab(boolean grab){
        if(grab){
            this.m_Grabber.setPosition(Robot4100Generation1_Settings.GRABBERSERVO_GRABPOS);
        }else{
            this.m_Grabber.setPosition(Robot4100Generation1_Settings.GRABBERSERVO_RESTPOS);
        }
    }

    public void setGrabberRotServoToOutside(boolean outside, double speed){
        if(outside){
            this.m_GrabberRot.setTargetPosition(Robot4100Generation1_Settings.GRABBERROTSERVO_OUTSIDEPOS, speed);
        }else{
            this.m_GrabberRot.setTargetPosition(Robot4100Generation1_Settings.GRABBERROTSERVO_INSIDEPOS, speed);
        }
    }

    public void setIntakeSystemStatus(IntakeSystemStatus status, double Speed){
        Speed = Math.abs(Speed);
        switch(status){
            case SUCK:
                this.m_IntakeLeft.setPower(Speed);
                this.m_IntakeRight.setPower(-Speed);
                break;
            case VOMIT:
                this.m_IntakeLeft.setPower(-Speed);
                this.m_IntakeRight.setPower(Speed);
                break;
            case STOP:
                this.m_IntakeLeft.setPower(0);
                this.m_IntakeRight.setPower(0);
                break;
        }
    }

    public void setOrientServoToOrient(boolean orient){
        if(orient){
            this.m_StoneOrientServo.setPosition(Robot4100Generation1_Settings.STONEORIENTSERVO_ORIENTPOS);
        }else{
            this.m_StoneOrientServo.setPosition(Robot4100Generation1_Settings.STONEORIENTSERVO_ZEROPOS);
        }
    }

    public IntakeSystemStatus getIntakeSystemStatus(){
        if(this.m_IntakeLeft.getPower() > 0) {
            return IntakeSystemStatus.SUCK;
        }else if(this.m_IntakeLeft.getPower() < 0){
            return IntakeSystemStatus.VOMIT;
        }else{
            return IntakeSystemStatus.STOP;
        }
    }

    @Override
    public void stop() {
        super.stop();
        this.getLinearSlide().deleteAllTasks();
        this.m_GrabberRot.stop();
    }

    @Override
    public void terminate() {
        super.terminate();
    }

    @Override
    public boolean isBusy() {
        return this.m_linearSlide.isBusy() || this.m_GrabberRot.isBusy() || super.isBusy();
    }

    @Override
    public void updateStatus() {
        super.updateStatus();
        this.m_linearSlide.updateStatus();
        this.m_GrabberRot.updateStatus();
    }
}
