package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.chassiscontrollers.OmniDrive;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;

public class Robot4100Generation1_LindaCore extends RobotCore {
    public enum IntakeSystemStatus{
        SUCK,
        VOMIT,
        STOP
    }
    private OmniDrive m_Chassis;
    private Servo m_DragServoL, m_DragServoR;
    private Servo m_Grabber, m_GrabberRot;
    private Servo m_StoneOrientServo;
    private RobotServoUsingMotor m_linearSlide;
    private DcMotor m_IntakeLeft, m_IntakeRight;

    public Robot4100Generation1_LindaCore(HardwareMap hardwares) {
        super("4100Generation1_LindaCore.log");
        RobotMotor
                LFMotor = new RobotMotorWithEncoder(hardwares.dcMotor.get("LF"),Robot4100Generation1_Settings.motorType),
                RFMotor = new RobotMotorWithEncoder(hardwares.dcMotor.get("RF"),Robot4100Generation1_Settings.motorType),
                LBMotor = new RobotMotorWithEncoder(hardwares.dcMotor.get("LB"),Robot4100Generation1_Settings.motorType),
                RBMotor = new RobotMotorWithEncoder(hardwares.dcMotor.get("RB"),Robot4100Generation1_Settings.motorType);
        RobotWheel
                LFWheel = new RobotWheel(new Robot2DPositionIndicator(-Robot4100Generation1_Settings.wheelPosition[0],Robot4100Generation1_Settings.wheelPosition[1],45),Robot4100Generation1_Settings.wheelRadius),
                RFWheel = new RobotWheel(new Robot2DPositionIndicator(Robot4100Generation1_Settings.wheelPosition[0],Robot4100Generation1_Settings.wheelPosition[1],-45),Robot4100Generation1_Settings.wheelRadius),
                LBWheel = new RobotWheel(new Robot2DPositionIndicator(-Robot4100Generation1_Settings.wheelPosition[0],-Robot4100Generation1_Settings.wheelPosition[1],135),Robot4100Generation1_Settings.wheelRadius),
                RBWheel = new RobotWheel(new Robot2DPositionIndicator(Robot4100Generation1_Settings.wheelPosition[0],-Robot4100Generation1_Settings.wheelPosition[1],-135),Robot4100Generation1_Settings.wheelRadius);
        RobotMotion
                LFMotion = new RobotMotion(new RobotMotorController(LFMotor,Robot4100Generation1_Settings.CHASSIS_TIMEOUTENABLE,Robot4100Generation1_Settings.CHASSIS_TIMEOUTFACTOR),LFWheel),
                RFMotion = new RobotMotion(new RobotMotorController(RFMotor,Robot4100Generation1_Settings.CHASSIS_TIMEOUTENABLE,Robot4100Generation1_Settings.CHASSIS_TIMEOUTFACTOR),RFWheel),
                LBMotion = new RobotMotion(new RobotMotorController(LBMotor,Robot4100Generation1_Settings.CHASSIS_TIMEOUTENABLE,Robot4100Generation1_Settings.CHASSIS_TIMEOUTFACTOR),LBWheel),
                RBMotion = new RobotMotion(new RobotMotorController(RBMotor,Robot4100Generation1_Settings.CHASSIS_TIMEOUTENABLE,Robot4100Generation1_Settings.CHASSIS_TIMEOUTFACTOR),RBWheel);
        m_Chassis = new OmniDrive(LFMotion,RFMotion,LBMotion,RBMotion,null);
        this.m_DragServoL = hardwares.servo.get("servoDragLeft");
        this.m_DragServoR = hardwares.servo.get("servoDragRight");
        this.m_Grabber = hardwares.servo.get("servoGrabber");
        this.m_GrabberRot = hardwares.servo.get("servoGrabberRot");
        RobotMotor LinearSlideMotor = new RobotMotorWithEncoder(hardwares.dcMotor.get("motorLinearSlide"),Robot4100Generation1_Settings.linearSlideMotorType);
        RobotMotorController linearSlideController = new RobotMotorController(LinearSlideMotor,Robot4100Generation1_Settings.LINEARSLIDE_TIMEOUTCONTROLENABLE,Robot4100Generation1_Settings.LINEARSLIDE_TIMEOUTFACTOR);
        this.m_linearSlide = new RobotServoUsingMotor(linearSlideController,Robot4100Generation1_Settings.LINEARSLIDE_START,Robot4100Generation1_Settings.LINEARSLIDE_MIN,Robot4100Generation1_Settings.LINEARSLIDE_MAX);

        this.m_IntakeLeft = hardwares.dcMotor.get("motorIntakeLeft");
        this.m_IntakeRight = hardwares.dcMotor.get("motorIntakeRight");

        this.m_StoneOrientServo = hardwares.servo.get("servoStoneOrient");
    }

    public RobotServoUsingMotor getLinearSlide(){
        return this.m_linearSlide;
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

    public void setGrabberRotServoToOutside(boolean outside){
        if(outside){
            this.m_GrabberRot.setPosition(Robot4100Generation1_Settings.GRABBERROTSERVO_OUTSIDEPOS);
        }else{
            this.m_GrabberRot.setPosition(Robot4100Generation1_Settings.GRABBERROTSERVO_INSIDEPOS);
        }
    }

    public void setIntakeSystemStatus(IntakeSystemStatus status){
        switch(status){
            case SUCK:
                this.m_IntakeLeft.setPower(Robot4100Generation1_Settings.INTAKEMOTOR_SPEED);
                this.m_IntakeRight.setPower(-Robot4100Generation1_Settings.INTAKEMOTOR_SPEED);
                break;
            case VOMIT:
                this.m_IntakeLeft.setPower(-Robot4100Generation1_Settings.INTAKEMOTOR_SPEED);
                this.m_IntakeRight.setPower(Robot4100Generation1_Settings.INTAKEMOTOR_SPEED);
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
        if(this.m_IntakeLeft.getPower() == Robot4100Generation1_Settings.INTAKEMOTOR_SPEED) {
            return IntakeSystemStatus.SUCK;
        }else if(this.m_IntakeLeft.getPower() == -Robot4100Generation1_Settings.INTAKEMOTOR_SPEED){
            return IntakeSystemStatus.VOMIT;
        }else{
            return IntakeSystemStatus.STOP;
        }
    }

    @Override
    public void stop() {
        this.m_Chassis.deleteAllTasks();
    }

    @Override
    public void terminate() {
        return;
    }

    @Override
    public RobotMotionSystem getChassis() {
        return this.m_Chassis;
    }

    @Override
    public boolean isBusy() {
        return this.m_linearSlide.isBusy();
    }

    @Override
    public void updateStatus() {
        this.m_Chassis.updateStatus();
        this.m_linearSlide.updateStatus();
    }
}
