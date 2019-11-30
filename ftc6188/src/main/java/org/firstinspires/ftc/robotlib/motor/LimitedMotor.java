package org.firstinspires.ftc.robotlib.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotlib.state.ToggleBoolean;

/*
A limited motor is a modified motor which has pre defined limits that when enabled will not permit the motor to travel passed them
The accuracy is poor and often times the motor will run passed the limits when the power is high but for the most part
it will protect sensitive equipment, such as linear slides, from over or under extending
 */
public class LimitedMotor extends ModifiedMotor
{
    // Limiting variables
    private int upperLimit;
    private int lowerLimit;
    private boolean limited;

    public LimitedMotor(DcMotor motor, int lowerLimit, int upperLimit, boolean limited)
    {
        super(motor);
        this.upperLimit = upperLimit;
        this.lowerLimit = lowerLimit;

        // the motors default state is to be limited unless the motor has no limits set
        this.limited = limited;
    }

    public LimitedMotor(DcMotor motor, int lowerLimit, int upperLimit) { this(motor, lowerLimit, upperLimit, true); }

    public LimitedMotor(DcMotor motor) { this(motor, 0, 0, false); }

    @Override
    public void setPower(double power)
    {
        // If both the upper limit and lower limit are the same (such as in default init) then the motor will not be constrained
        limited = upperLimit != lowerLimit;

        if (limited)
        {
            if ((power > 0 && motor.getCurrentPosition() >= upperLimit)
                    || (power < 0 && motor.getCurrentPosition() <= lowerLimit)) { motor.setPower(0); }
            else { motor.setPower(power); }
        }
        else { motor.setPower(power); }
    }

    public void setLimited(boolean limited)
    {
        this.limited = limited;

        // runs the two limit checks to ensure that despite the user enabling limits the robot can be limited
        if (limited)
        {
            setMode(motor.getMode());
            setPower(0);
        }
    }

    public boolean isLimited() { return limited; }

    public void setUpperLimit(int upperLimit) { this.upperLimit = upperLimit; }

    public int getUpperLimit() { return upperLimit; }

    public void setLowerLimit(int lowerLimit) { this.lowerLimit = lowerLimit; }

    public int getLowerLimit() { return lowerLimit; }

    @Override
    public void setMode(DcMotor.RunMode runMode)
    {
        motor.setMode(runMode);

        // Limited can only be enabled if the motor is set to run using encoders
        limited = runMode == DcMotor.RunMode.RUN_USING_ENCODER;
    }
}
