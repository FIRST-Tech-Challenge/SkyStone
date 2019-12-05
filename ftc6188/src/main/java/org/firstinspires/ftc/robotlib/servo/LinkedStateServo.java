package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotlib.state.ServoState;

/*
Much like a linked servo only this responds to ServoState enum
 */
public class LinkedStateServo extends ModifiedServo
{
    // servo two holder
    private Servo servoTwo;

    // positional holders
    private double[] positionsOne;
    private double[] positionsTwo;

    private ServoState servoState;

    public LinkedStateServo(StateServo servo, StateServo servoTwo)
    {
        super(servo);
        this.servoTwo = servoTwo;

        positionsOne = servo.getPositions();
        positionsTwo = servoTwo.getPositions();
    }

    public void setPosition(ServoState servoState)
    {
        if (servoState != ServoState.UNKNOWN)
        {
            this.servo.setPosition(positionsOne[servoState.getStateLevel()]);
            servoTwo.setPosition(positionsTwo[servoState.getStateLevel()]);
        }
    }

    public ServoState getServoState() { return servoState; }

    public double[] getPositionsOne() { return positionsOne; }

    public double[] getPositionsTwo() { return positionsTwo; }
}
