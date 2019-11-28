package org.firstinspires.ftc.robotlib.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

public class LimitedMotor extends ModifiedMotor
{
    // Limiting variables
    private int upperLimit;
    private int lowerLimit;
    private boolean limited;

    public LimitedMotor(DcMotor motor, int lowerLimit, int upperLimit)
    {
        super(motor);
        this.upperLimit = upperLimit;
        this.lowerLimit = lowerLimit;
        limited = false;
    }

    public LimitedMotor(DcMotor motor) { this(motor, 0, 0); }

    @Override
    public void setPower(double power)
    {
        if (limited)
        {
            if ((power > 0 && motor.getCurrentPosition() >= upperLimit)
                    || (power < 0 && motor.getCurrentPosition() <= lowerLimit)) { motor.setPower(0); }
            else { motor.setPower(power); }
        }
        else { motor.setPower(power); }
    }

    public void setLimited(boolean limited) { this.limited = limited; }

    public boolean isLimited() { return limited; }

    public void setUpperLimit(int upperLimit) { this.upperLimit = upperLimit; }

    public int getUpperLimit() { return upperLimit; }

    public void setLowerLimit(int lowerLimit) { this.lowerLimit = lowerLimit; }

    public int getLowerLimit() { return lowerLimit; }

    @Override
    public void setMode(DcMotor.RunMode runMode)
    {
        motor.setMode(runMode);
        limited = runMode == DcMotor.RunMode.RUN_USING_ENCODER;
    }
}
