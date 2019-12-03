package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.ArmSystem;

import java.util.EnumMap;

@TeleOp(name = "Just The Arm System", group="TeleOp")
public class JustTheArmSystem extends OpMode {
    public ArmSystem armSystem;
    public void init() {
        EnumMap<ArmSystem.ServoNames, Servo> servoEnumMap = new EnumMap<ArmSystem.ServoNames, Servo>(ArmSystem.ServoNames.class);
        servoEnumMap.put(ArmSystem.ServoNames.GRIPPER, hardwareMap.get(Servo.class, "GRIPPER"));
        servoEnumMap.put(ArmSystem.ServoNames.ELBOW, hardwareMap.get(Servo.class, "ELBOW"));
        servoEnumMap.put(ArmSystem.ServoNames.WRIST, hardwareMap.get(Servo.class, "WRIST"));
        servoEnumMap.put(ArmSystem.ServoNames.PIVOT, hardwareMap.get(Servo.class, "PIVOT"));
        armSystem = new ArmSystem(
                servoEnumMap,
                hardwareMap.get(DcMotor.class, "SLIDER_MOTOR"),
                hardwareMap.get(DigitalChannel.class, "SLIDER_SWITCH"), false);
    }

    public void loop() {
        /*
    public String run(boolean home, boolean capstone, boolean west, boolean east, boolean north, boolean south,
                      boolean up, boolean down, boolean gripperButton, boolean assist,
                      double sliderSpeed, double armSpeed, boolean fastMode) {
         */
        String armReturn = armSystem.run(gamepad2.x, gamepad2.y, gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.dpad_up,
                gamepad2.dpad_down, gamepad2.right_bumper, gamepad2.left_bumper, gamepad2.a,
                true,1, 0.005, true);
        telemetry.addData("", armReturn);
    }
}
