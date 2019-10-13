package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.RobotMap.HookServo;

import java.util.HashMap;
import java.util.Map;

public class Hook {
    HashMap<HookServo, Servo> servos;

    public Hook(HardwareMap hardwareMap, HashMap<RobotMap.HookServo, String> servoNames) {
        servos = new HashMap<>();
        for (Map.Entry<RobotMap.HookServo, String> servoName : servoNames.entrySet()) {
            servos.put(servoName.getKey(), hardwareMap.servo.get(servoName.getValue()));
        }
    }

    public void setPosition(double position) {
        servos.get(HookServo.MAIN).setPosition(position);
    }

    public double getPosition() {
        return servos.get(HookServo.MAIN).getPosition();
    }

    public void runServo(double power) {
        servos.get(HookServo.MAIN).setPosition(servos.get(HookServo.MAIN).getPosition() + power);
    }

}