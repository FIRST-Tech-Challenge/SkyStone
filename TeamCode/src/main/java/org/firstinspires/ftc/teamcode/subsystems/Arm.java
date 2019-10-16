package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.subsystems.RobotMap.ArmMotor;
import static org.firstinspires.ftc.teamcode.subsystems.RobotMap.ArmServo;


public class Arm {
    int robotHeight;
    int robotLength;
    int blockHeight;
    int foundationHeight;
    int armLength;
    HashMap<ArmMotor, DcMotor> motors;
    HashMap<ArmServo, Servo> servos;

    public Arm(HardwareMap hardwareMap, HashMap<ArmMotor, String> motorNames, HashMap<ArmServo, String> servoNames) {
        motors = new HashMap<>();
        for (Map.Entry<ArmMotor, String> motorName : motorNames.entrySet()) {
            motors.put(motorName.getKey(), hardwareMap.dcMotor.get(motorName.getValue()));
        }
        servos = new HashMap<>();
        for (Map.Entry<ArmServo, String> servoName : servoNames.entrySet()) {
            servos.put(servoName.getKey(), hardwareMap.servo.get(servoName.getValue()));
        }
    }

    public void setArm(Chassis chassis, int level) {
        double distance;
        double armAngle;
        int heightDif = Math.abs(robotHeight - (blockHeight * level + foundationHeight));
        armAngle = Math.acos((double) heightDif / (double) armLength);
        distance = Math.sqrt((armLength * armLength) - (heightDif * heightDif)) + robotLength;
    }

    public void setPosition(double position, ArmServo servo) {
        servos.get(servo).setPosition(position);
    }

    public double getPosition(ArmServo servo) {
        return servos.get(servo).getPosition();
    }

    public void runServo(ArmServo servo, double power) {
        setPosition(getPosition(servo) + power, servo);
    }

    public void runArm(HashMap<ArmServo, Double> positions, HashMap<ArmMotor, Double> powers) {
        for (Map.Entry<ArmMotor, Double> power : powers.entrySet()) {
            motors.get(power.getKey()).setPower(power.getValue());
        }
        for (Map.Entry<ArmServo, Double> position : positions.entrySet()) {
            runServo(position.getKey(), position.getValue());
        }
    }
}
