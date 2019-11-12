package org.firstinspires.ftc.robotlib.armsystem;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.motor.LimitedMotor;

public interface DualLinearSlideSystem
{
    void setVerticalPower(double power);
    void setHorizontalPower(double power);
    LimitedMotor getVerticalLimitedMotor();
    LimitedMotor getHorizontalLimitedMotor();
    DcMotor getVerticalMotor();
    DcMotor getHorizontalMotor();
}
