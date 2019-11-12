package org.firstinspires.ftc.robotlib.armsystem;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.motor.LimitedMotor;

public class FieldGoalArmSystem implements DualLinearSlideSystem
{
    private LimitedMotor armVerticalSlide; // motor one
    private LimitedMotor armHorizontalSlide; // motor two

    public FieldGoalArmSystem(LimitedMotor verticalSlide, LimitedMotor horizontalSlide)
    {
        this.armVerticalSlide = verticalSlide;
        this.armHorizontalSlide = horizontalSlide;
    }

    @Override
    public void setVerticalPower(double power) { armVerticalSlide.setPower(power); }

    @Override
    public void setHorizontalPower(double power) { armHorizontalSlide.setPower(power); }

    @Override
    public LimitedMotor getVerticalLimitedMotor() { return armVerticalSlide; }

    @Override
    public LimitedMotor getHorizontalLimitedMotor() { return armHorizontalSlide; }

    @Override
    public DcMotor getVerticalMotor() { return armVerticalSlide.getMotor(); }

    @Override
    public DcMotor getHorizontalMotor() { return armHorizontalSlide.getMotor(); }
}
