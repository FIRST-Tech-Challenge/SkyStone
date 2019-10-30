package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StoneManipulator {

    private static StoneManipulator instance = null;
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    public static synchronized StoneManipulator getInstance() {
        return instance != null ? instance : (instance = new StoneManipulator());
    }

    private StoneManipulator() {}

    public void init(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class,"");//This is intake left motor
        rightMotor = hardwareMap.get(DcMotorEx.class,"");// This is intake right motor

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);


    }

    public void idle(){

    }

    public void zero(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void intake(){
        leftMotor.setPower(-1);
        rightMotor.setPower(-1);
    }
}
