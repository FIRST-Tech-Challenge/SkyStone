package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.subsystems.RobotMap.HookServo;

import java.util.HashMap;

public class Hook {
    HashMap<HookServo, Servo> servos;

    public Hook(HashMap<HookServo, Servo> servos) {
        this.servos = servos;
    }


    public void setPosition(double position) {
        servos.get(HookServo.MAIN).setPosition(position);
    }

    public double getPosition(){
       return servos.get(HookServo.MAIN).getPosition();
    }
}