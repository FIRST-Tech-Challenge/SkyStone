package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

public class Subsystem {
    HashMap<String, DcMotor> motors;
    HashMap<String, Servo> servos;
    public Subsystem(HardwareMap hardwareMap) throws FileNotFoundException {

    }

    public Subsystem(HashMap<String, DcMotor> motors, HashMap<String, Servo> servos) {
        this.motors = motors;
        this.servos = servos;
    }

    public HashMap<String, DcMotor> getMotors() {
        return motors;
    }

    public void setMotors(HashMap<String, DcMotor> motors) {
        this.motors = motors;
    }

    public HashMap<String, Servo> getServos() {
        return servos;
    }

    public void setServos(HashMap<String, Servo> servos) {
        this.servos = servos;
    }

    public void setMotorPowers(HashMap<String, Double> powers) {
        for(Map.Entry<String, Double> power : powers.entrySet())
            motors.get(power.getKey()).setPower(power.getValue());
    }

    public void setServoPositions(HashMap<String, Double> positions){
        for(Map.Entry<String, Double> position : positions.entrySet())
            servos.get(position.getKey()).setPosition(position.getValue());
    }

}
