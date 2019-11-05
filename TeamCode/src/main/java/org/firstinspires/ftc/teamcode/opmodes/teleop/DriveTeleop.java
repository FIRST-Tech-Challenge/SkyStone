package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

import java.util.EnumMap;

@TeleOp(name = "Real Teleop", group="TeleOp")
public class DriveTeleop extends BaseOpMode {

    // We need to have arm code in this OpMode because we don't want to calibrate the arm
    // in BaseOpMode

    private ArmSystem armSystem;
    // Use the following variables to detect if their respective bumpers have been pressed the
    // previous loop. Otherwise, hitting a bumper will increase the queued height by a like 30.
    boolean m_right = false;
    boolean m_left = false;
    boolean m_gripper = false; // Gripper button`

    public void init() {
        super.init();
        EnumMap<ArmSystem.ServoNames, Servo> servoEnumMap = new EnumMap<ArmSystem.ServoNames, Servo>(ArmSystem.ServoNames.class);
        servoEnumMap.put(ArmSystem.ServoNames.GRIPPER, hardwareMap.get(Servo.class, "gripper"));
        servoEnumMap.put(ArmSystem.ServoNames.ELBOW, hardwareMap.get(Servo.class, "elbow"));
        servoEnumMap.put(ArmSystem.ServoNames.WRIST, hardwareMap.get(Servo.class, "wrist"));
        servoEnumMap.put(ArmSystem.ServoNames.PIVOT, hardwareMap.get(Servo.class, "pivot"));
        armSystem = new ArmSystem(
                servoEnumMap,
                hardwareMap.get(DcMotor.class, "slider_motor"),
                hardwareMap.get(DigitalChannel.class, "slider_switch"), false);
        armSystem.calibrate();
    }

    public void start() {
        armSystem.setSliderHeight(1);
        armSystem.movePresetPosition(ArmSystem.Position.POSITION_HOME);
    }

    public void loop(){
        float rx = (float) Math.pow(gamepad1.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad1.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad1.left_stick_y, 3);
        driveSystem.drive(rx, lx, -ly, gamepad1.x);
        spinnySystem.spin(gamepad1.left_bumper, gamepad1.right_bumper);
        latchSystem.run(gamepad2.x, gamepad2.y);
        armSystem.run(gamepad2.b, gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.right_bumper, gamepad2.left_bumper, gamepad2.a,
                true,1, 0.005);
    }
}