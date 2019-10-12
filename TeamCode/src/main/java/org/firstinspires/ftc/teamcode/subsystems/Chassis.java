package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;
import static org.firstinspires.ftc.teamcode.subsystems.RobotMap.ChassisMotor;

public class Chassis{
    //Vars
    HashMap<ChassisMotor, DcMotor> motors;
    //Constructors
    public Chassis() {
        motors = new HashMap<ChassisMotor, DcMotor>();
    }
    public Chassis(HardwareMap hardwareMap, String motorFilePath) throws IOException {
        motors = new HashMap<ChassisMotor, DcMotor>();
        Scanner motorScanner = new Scanner(new File(motorFilePath));
    }
    public Chassis(HardwareMap hardwareMap, HashMap<ChassisMotor, String> motorsNames) {
        motors = new HashMap<>();
        for(Map.Entry<ChassisMotor, String> motorName : motorsNames.entrySet()){
            motors.put(motorName.getKey(),hardwareMap.dcMotor.get(motorName.getValue()));
        }
    }
    //Methods
    public void setMotors(HashMap<ChassisMotor, Double> powers){
        for(Map.Entry<ChassisMotor, Double> power : powers.entrySet()){
            motors.get(power.getKey()).setPower(power.getValue());
        }
    }
}
