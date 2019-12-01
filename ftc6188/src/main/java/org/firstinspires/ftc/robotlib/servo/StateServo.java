package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotlib.state.ServoState;

/*
A StateServo is a servo whose position can be set by specifying an enum state
 */
public class StateServo extends ModifiedServo
{
    private double[] positions;

    public StateServo(Servo servo, double stowedPosition, double upPosition, double downPosition)
    {
        super(servo);
        positions = new double[] {stowedPosition, upPosition, downPosition};
    }

    public StateServo(Servo servo) { this(servo, 1, 0, 1); }

    public void setPosition(ServoState servoState)
    {
        if (servoState != ServoState.UNKNOWN)
        {
            setPosition(positions[servoState.getLevel()]);
        }
    }

    public ServoState getState()
    {
        for (int index = positions.length-1; index > 0; index--)
        {
            if (positions[index] == getPosition())
            {
                return ServoState.getServoStateFromInt(index);
            }
        }
        return ServoState.UNKNOWN;
    }

    public void setPositions(double[] positions) { this.positions = positions; }

    public double[] getPositions() { return positions; }
}
