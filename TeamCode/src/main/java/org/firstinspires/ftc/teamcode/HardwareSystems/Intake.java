package org.firstinspires.ftc.teamcode.HardwareSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

public class Intake {

    public Intake(){}

    public void initialize(){

        RobotHardware.intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RobotHardware.intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /* This messed up odometer so not doing it
        RobotHardware.intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RobotHardware.intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
         */

    }


    public void setPower(double power){ //Setting a positive power should intake

        RobotHardware.intakeRight.setPower(power);
        RobotHardware.intakeLeft.setPower(-power);//Since not reversed up there

    }
}
