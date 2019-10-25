package org.firstinspires.ftc.teamcode.alek_wang;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.chassiscontrollers.OmniDrive;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark2964;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class Robot4100Core_Alek extends RobotCore {
    private static final MotorType SERVO_MOTOR_TYPE = new AndyMark2964();
    private static final boolean TIME_CONTROL_ENABLED = true;
    private static final double TIME_CONTROL_FACTOR = 1.5;
    private static final double INITIALIZED_POSITION = 0.0;
    private static final double MIN_POSITION = 0.0, MAX_POSITION = 1.0;

    private RobotServoUsingMotor m_ServoUsingMotor;
    private OmniDrive m_OmniDrive;

    public Robot4100Core_Alek(HardwareMap Hardware) {
        super("Robot4100Core_Alek.log",Hardware);
        DcMotor ServoUsingMotor_DC = Hardware.dcMotor.get("MotorServo");
        RobotMotorController ServoUsingMotorController = new RobotMotorController(
                new RobotMotorWithEncoder(ServoUsingMotor_DC,SERVO_MOTOR_TYPE),
                TIME_CONTROL_ENABLED,
                TIME_CONTROL_FACTOR
        );
        this.m_ServoUsingMotor = new RobotServoUsingMotor(
                ServoUsingMotorController,
                INITIALIZED_POSITION,
                MIN_POSITION,
                MAX_POSITION
        );
    }

    public RobotServoUsingMotor getServoUsingMotor(){
        return this.m_ServoUsingMotor;
    }

    @Override
    public void stop() {
        this.m_ServoUsingMotor.deleteAllTasks();
    }

    @Override
    public void terminate() {

    }

    @Override
    public RobotMotionSystem getChassis() {
        return null;
    }

    @Override
    public boolean isBusy() {
        return this.m_ServoUsingMotor.isBusy();
    }

    @Override
    public void updateStatus() {
        this.m_ServoUsingMotor.updateStatus();
    }
}
