package org.firstinspires.ftc.robotlib.armsystem;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.motor.EncoderMotor;
import org.firstinspires.ftc.robotlib.motor.LimitedMotor;

public class FieldGoalArmSystem
{
    private LimitedMotor armVerticalSlide; // motor one
    private LimitedMotor armHorizontalSlide; // motor two

    public FieldGoalArmSystem(LimitedMotor armVerticalSlide, LimitedMotor armHorizontalSlide)
    {
        this.armVerticalSlide = armVerticalSlide;
        this.armHorizontalSlide = armHorizontalSlide;
    }

    public FieldGoalArmSystem(DcMotor armVerticalSlide, DcMotor armHorizontalSlide) { this(new LimitedMotor(armVerticalSlide), new LimitedMotor(armHorizontalSlide)); }

    public void setVerticalPower(double power) { armVerticalSlide.setPower(power); }

    public void setHorizontalPower(double power) { armHorizontalSlide.setPower(power); }

    public void setLimited(boolean limited)
    {
        armVerticalSlide.setLimited(limited);
        armHorizontalSlide.setLimited(limited);
    }

    public LimitedMotor getVerticalLimitedMotor() { return armVerticalSlide; }

    public LimitedMotor getHorizontalLimitedMotor() { return armHorizontalSlide; }

    public void armAutoPosition(int horizTarget, int vertTarget, double power)
    {
        EncoderMotor armEncoderHorizontalSlide = new EncoderMotor(armHorizontalSlide);
        EncoderMotor armEncoderVerticalSlide = new EncoderMotor(armVerticalSlide);

        armEncoderHorizontalSlide.setTargetPosition(horizTarget);
        armEncoderVerticalSlide.setTargetPosition(vertTarget);

        armEncoderHorizontalSlide.setPower(power);
        armEncoderVerticalSlide.setPower(power);

        while (armEncoderHorizontalSlide.isEncoderBusy() || armEncoderVerticalSlide.isEncoderBusy()) {}

        armEncoderHorizontalSlide.setPower(0);
        armEncoderVerticalSlide.setPower(0);
    }
}
