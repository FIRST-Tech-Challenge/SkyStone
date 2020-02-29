package org.firstinspires.ftc.teamcode.HardwareSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

public class Intake {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    public Intake(LinearOpMode opMode, RobotHardware hardware){
        this.hardware = hardware;
        this.opMode = opMode;
    }

    public void initialize(){

        hardware.intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /* This messed up odometer so not doing it
        RobotHardware.intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RobotHardware.intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
         */

    }


    public void setPower(double power){ //Setting a positive power should intake

        hardware.intakeRight.setPower(power);
        hardware.intakeLeft.setPower(-power);//Since not reversed up there

    }
}
