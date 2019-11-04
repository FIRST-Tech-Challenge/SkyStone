package org.darbots.darbotsftclib.libcore.common_robotcores;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.chassiscontrollers.OmniDrive;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class OmniChassisRobotCore extends RobotCore {
    private RobotMotion m_LeftTopMotion, m_LeftBottomMotion, m_RightTopMotion, m_RightBottomMotion;
    private OmniDrive m_Drive;

    public OmniChassisRobotCore(String logName, HardwareMap Hardwares, double[] WheelPositionRobotAxis_PositiveValues, double WheelRadius, String LeftTopConfigName, String LeftBottomConfigName, String RightTopConfigName, String RightBottomConfigName, MotorType ChassisMotorType, boolean ChassisTimeoutEnabled, double ChassisTimeoutFactor) {
        super(logName,Hardwares);
        DcMotor m_LeftTopDC = Hardwares.dcMotor.get(LeftTopConfigName);
        DcMotor m_LeftBottomDC = Hardwares.dcMotor.get(LeftBottomConfigName);
        DcMotor m_RightTopDC = Hardwares.dcMotor.get(RightTopConfigName);
        DcMotor m_RightBottomDC = Hardwares.dcMotor.get(RightBottomConfigName);
        m_LeftTopDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_LeftBottomDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_RightTopDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_RightBottomDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotWheel m_LeftTopWheel = new RobotWheel(new Robot2DPositionIndicator(-WheelPositionRobotAxis_PositiveValues[0],WheelPositionRobotAxis_PositiveValues[1],45),WheelRadius);
        RobotWheel m_RightTopWheel = new RobotWheel(new Robot2DPositionIndicator(WheelPositionRobotAxis_PositiveValues[0],WheelPositionRobotAxis_PositiveValues[1],-45),WheelRadius);
        RobotWheel m_LeftBottomWheel = new RobotWheel(new Robot2DPositionIndicator(-WheelPositionRobotAxis_PositiveValues[0],-WheelPositionRobotAxis_PositiveValues[1],135),WheelRadius);
        RobotWheel m_RightBottomWheel = new RobotWheel(new Robot2DPositionIndicator(WheelPositionRobotAxis_PositiveValues[0],-WheelPositionRobotAxis_PositiveValues[1],-135),WheelRadius);
        this.m_LeftTopMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_LeftTopDC,ChassisMotorType),ChassisTimeoutEnabled,ChassisTimeoutFactor),m_LeftTopWheel);
        this.m_LeftBottomMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_LeftBottomDC,ChassisMotorType),ChassisTimeoutEnabled,ChassisTimeoutFactor),m_LeftBottomWheel);
        this.m_RightTopMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_RightTopDC,ChassisMotorType),ChassisTimeoutEnabled,ChassisTimeoutFactor),m_RightTopWheel);
        this.m_RightBottomMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_RightBottomDC,ChassisMotorType),ChassisTimeoutEnabled,ChassisTimeoutFactor),m_RightBottomWheel);
        this.m_Drive = new OmniDrive(this.m_LeftTopMotion,this.m_RightTopMotion,this.m_LeftBottomMotion,this.m_RightBottomMotion,null);
    }

    @Override
    public void stop() {
        this.m_Drive.deleteAllTasks();
    }

    @Override
    public void terminate() {
        return;
    }

    @Override
    public RobotMotionSystem getChassis() {
        return this.m_Drive;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void updateStatus() {
        m_Drive.updateStatus();
    }
}
