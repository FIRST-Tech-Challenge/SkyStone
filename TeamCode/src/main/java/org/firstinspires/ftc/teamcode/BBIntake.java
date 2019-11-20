package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BBIntake {

    private DcMotor _leftIntakeMotor;
    private DcMotor _rightIntakeMotor;

    public void init(HardwareMap hwmap){

        _leftIntakeMotor = hwmap.get(DcMotor.class, "left_intake");
        _rightIntakeMotor = hwmap.get(DcMotor.class, "right_intake");
    }

    public void Start()
    {

        _leftIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _rightIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _leftIntakeMotor.setPower(1);
        _rightIntakeMotor.setPower(1);
    }

    public void Stop()
    {
        _leftIntakeMotor.setPower(0);
        _rightIntakeMotor.setPower(0);
    }

    public void Reverse()
    {

        _leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _rightIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _leftIntakeMotor.setPower(1);
        _rightIntakeMotor.setPower(1);
    }
}
