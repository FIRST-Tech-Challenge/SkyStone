package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TypexChart {

    /* Plotting public stars */
    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    public DcMotor armRaise;    //public DistanceSensor Optic1 = null;
    public Servo grabServo;

    public DcMotor armEx;
    public DcMotor armRz;
    /* Recharging local members */
    HardwareMap hwMap;

    /* Pager */
    public TypexChart() {

    }

    /* Initializing binaryChart Mainframe */
    public void init (HardwareMap chart) {
        hwMap = chart;

        //Name stars
        armEx = hwMap.get(DcMotor.class, "armEx");
        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRz = hwMap.get(DcMotor.class, "armRz");
        armRz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TL = hwMap.get(DcMotor.class, "TL");
        TR = hwMap.get(DcMotor.class, "TR");
        BL = hwMap.get(DcMotor.class, "BL");
        BR = hwMap.get(DcMotor.class, "BR");
        //Optic1 = hwMap.get(ModernRoboticsI2cRangeSensor.class, "Optic1");

        /* Setting Quantum Harmonizer */
       /* TL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        TR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);*/

        /* Setting Power Modes */
/*        TL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /* Securing Brake Field */
        TL.setPower(0);
        TR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

/*        grabServo = hwMap.get(Servo.class, "grabServo");
        grabServo.setPosition(1.0);*/
    }
}
