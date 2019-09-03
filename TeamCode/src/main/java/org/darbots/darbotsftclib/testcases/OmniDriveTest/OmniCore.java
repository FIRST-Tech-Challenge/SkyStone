package org.darbots.darbotsftclib.testcases.OmniDriveTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.chassiscontrollers.OmniDrive;
import org.darbots.darbotsftclib.libcore.motortypes.RevHDHex20Motor;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class OmniCore extends RobotCore {
    private static double[] WHEEL_POS = {15,15}; //{X, Z}
    private static double WHEEL_RADIUS = 5;
    private static boolean TIMEOUT_ENABLED = true;
    private static double TIMEOUT_FACTOR = 1.5;
    private static MotorType MOTOR_TYPE = new RevHDHex20Motor();


    private RobotMotion m_LeftTopMotion, m_LeftBottomMotion, m_RightTopMotion, m_RightBottomMotion;
    private OmniDrive m_Drive;

    public OmniCore(HardwareMap Hardwares) {
        super("OmniDriveTest.log");
        this.getLogger().setDebugOn(true);
        DcMotor m_LeftTopDC = Hardwares.dcMotor.get("LT");
        DcMotor m_LeftBottomDC = Hardwares.dcMotor.get("LB");
        DcMotor m_RightTopDC = Hardwares.dcMotor.get("RT");
        DcMotor m_RightBottomDC = Hardwares.dcMotor.get("RB");
        RobotWheel m_LeftTopWheel = new RobotWheel(new Robot2DPositionIndicator(-WHEEL_POS[0],WHEEL_POS[1],45),WHEEL_RADIUS);
        RobotWheel m_RightTopWheel = new RobotWheel(new Robot2DPositionIndicator(WHEEL_POS[0],WHEEL_POS[1],-45),WHEEL_RADIUS);
        RobotWheel m_LeftBottomWheel = new RobotWheel(new Robot2DPositionIndicator(-WHEEL_POS[0],-WHEEL_POS[1],135),WHEEL_RADIUS);
        RobotWheel m_RightBottomWheel = new RobotWheel(new Robot2DPositionIndicator(WHEEL_POS[0],-WHEEL_POS[1],-135),WHEEL_RADIUS);
        this.m_LeftTopMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_LeftTopDC,MOTOR_TYPE),TIMEOUT_ENABLED,TIMEOUT_FACTOR),m_LeftTopWheel);
        this.m_LeftBottomMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_LeftBottomDC,MOTOR_TYPE),TIMEOUT_ENABLED,TIMEOUT_FACTOR),m_LeftBottomWheel);
        this.m_RightTopMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_RightTopDC,MOTOR_TYPE),TIMEOUT_ENABLED,TIMEOUT_FACTOR),m_RightTopWheel);
        this.m_RightBottomMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_RightBottomDC,MOTOR_TYPE),TIMEOUT_ENABLED,TIMEOUT_FACTOR),m_RightBottomWheel);
        this.m_Drive = new OmniDrive(this.m_LeftTopMotion,this.m_RightTopMotion,this.m_LeftBottomMotion,this.m_RightBottomMotion,null);
    }

    @Override
    public void stop() {
        this.m_Drive.deleteAllTasks();
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
