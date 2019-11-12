package org.firstinspires.ftc.robotlib.motor;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

public class LimitedMotor
{
    private DcMotor motor;
    private int upperLimit;
    private int lowerLimit;
    private ToggleBoolean limited;

    LimitedMotor(DcMotor motor, int upperLimit, int lowerLimit)
    {
        this.motor = motor;
        this.upperLimit = upperLimit;
        this.lowerLimit = lowerLimit;
        limited = new ToggleBoolean((motor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER));
    }

    public void setPower(double power)
    {
        if (limited.output())
        {
            if (power > 0)
            {
                if (!(motor.getCurrentPosition() >= upperLimit))
                {
                    motor.setPower(power);
                }
            }
            else if (power < 0)
            {
                if (!(motor.getCurrentPosition() <= lowerLimit))
                {
                    motor.setPower(power);
                }
            }
            else
            {
                motor.setPower(power);
            }
        }
        else
        {
            motor.setPower(power);
        }
    }

    public void setLimited(boolean input)
    {
        limited.input(input);
    }

    public void setMode(DcMotor.RunMode runMode)
    {
        motor.setMode(runMode);
        limited.input(runMode == DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public DcMotor getMotor()
    {
        return motor;
    }

    public double getPower()
    {
        return motor.getPower();
    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
    }
}
