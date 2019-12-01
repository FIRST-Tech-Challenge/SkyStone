package org.firstinspires.ftc.robotlib.servo;

import org.firstinspires.ftc.robotlib.state.ServoState;

/*
Much like a linked servo only this responds to ServoState enum
 */
public class LinkedStateServo
{
    // StateServos
    private StateServo servoOne;
    private StateServo servoTwo;

    private double[] positions;
    private boolean oppositeFace;

    public LinkedStateServo(StateServo servoOne, StateServo servoTwo, double stowedPosition, double upPosition, double downPosition, boolean oppositeFace)
    {
        this.servoOne = servoOne;
        this.servoTwo = servoTwo;

        positions = new double[] {stowedPosition, upPosition, downPosition};
        this.oppositeFace = oppositeFace;
    }

    public void setPosition(ServoState servoState)
    {
        if (oppositeFace)
        {
            servoOne.setPosition(positions[servoState.getLevel()]);
            servoTwo.setPosition(positions[positions.length-servoState.getLevel()]);
        }
        else
        {
            servoOne.setPosition(positions[servoState.getLevel()]);
            servoTwo.setPosition(positions[servoState.getLevel()]);
        }
    }

    public String getStateString() { return "Servo One: " + servoOne.getState() + " Servo Two: " + servoTwo.getState(); }
}
