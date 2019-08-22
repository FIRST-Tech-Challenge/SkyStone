package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TypexChart {

    /* Plotting public stars */
    public DcMotor TL = null;
    public DcMotor TR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    //public DistanceSensor Optic1 = null;

    /* Recharging local members */
    HardwareMap hwMap = null;

    /* Pager */
    public TypexChart() {

    }

    /* Initializing binaryChart Mainframe */
    public void init (HardwareMap chart) {
        hwMap = chart;

        //Name stars
        TL = hwMap.get(DcMotor.class, "TL");
        TR = hwMap.get(DcMotor.class, "TR");
        BL = hwMap.get(DcMotor.class, "BL");
        BR = hwMap.get(DcMotor.class, "BR");
        //Optic1 = hwMap.get(ModernRoboticsI2cRangeSensor.class, "Optic1");

        /* Setting Quantum Harmonizer */
        TL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        TR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        /* Setting Power Modes */
        TL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* Securing Brake Field */
        TL.setPower(0);
        TR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}
