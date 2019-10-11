package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

public class Chassis{
    //Vars
    HashMap<String, DcMotor> motors;
    //Constructors
    public Chassis(HardwareMap hardwareMap, String motorFile) throws IOException {
        motors = new HashMap<String, DcMotor>();
        Scanner motorScanner = new Scanner(new File(motorFile));
        while(motorScanner.hasNextLine()) {
            String[] motorParts = motorScanner.nextLine().split(",");
            motors.put(motorParts[0], hardwareMap.dcMotor.get(motorParts[1]));
        }
    }
    public Chassis(HardwareMap hardwareMap, String[][] motorNames) {
        motors = new HashMap<String, DcMotor>();
        for(String[] motorName : motorNames) {
            motors.put(motorName[0], hardwareMap.dcMotor.get(motorName[1]));
        }
    }
    //Methods
    public void setMotors(HashMap<String, Double> powers){
        for(Map.Entry<String, Double> power : powers.entrySet()){
            motors.get(power.getKey()).setPower(power.getValue());
        }
    }
}