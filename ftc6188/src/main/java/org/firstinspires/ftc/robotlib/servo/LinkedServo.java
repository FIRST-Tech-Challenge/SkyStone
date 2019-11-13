package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;
/*
This class defines two servos who are intended to operate as one
it really just simplifies the programming later since we then only have to set one position instead of two
 */
public class LinkedServo
{
    private Servo servoOne;
    private Servo servoTwo;

    private boolean oppositeFace;
    private double position;

    public LinkedServo(Servo servoOne, Servo servoTwo, boolean oppositeFace, double position)
    {
        this.servoOne = servoOne;
        this.servoTwo = servoTwo;
        this.oppositeFace = oppositeFace;
        this.position = position;
    }

    public LinkedServo(Servo servoOne, Servo servoTwo) { this(servoOne, servoTwo, (servoOne.getDirection() == servoTwo.getDirection()), 0); }

    // Operates just like the servo.setPosition function but does it for both, handles the weird math defining two servos
    public void setPosition(double position)
    {
        this.position = position;
        double position2 = position;
        if (oppositeFace)
        {
            position2 = -position;
            if (servoTwo.getPosition() - position < 0)
            {
                position2 += 360;
            }
            else if (servoTwo.getPosition() - position > 360)
            {
                position2 -= 360;
            }
        }

        servoTwo.setPosition(position2);
        servoOne.setPosition(position);
        // 0 is the down position while 1 is up, for some reason
    }

    public void setPositionVariable(double position)
    {
        this.position = position;
    }

    public void updateServoOne()
    {
        servoOne.setPosition(position);
    }

    public void updateServoTwo()
    {
        double position2 = position;
        if (oppositeFace)
        {
            position2 = -position;
            if (servoTwo.getPosition() - position < 0)
            {
                position2 += 360;
            }
            else if (servoTwo.getPosition() - position > 360)
            {
                position2 -= 360;
            }
        }

        servoTwo.setPosition(position2);
    }

    public double getPosition() { return position; }

    // Only for telemetry purposes
    public String getActual()
    {
        return ("One: " + servoOne.getPosition() + " Two: " + servoTwo.getPosition());
    }
}
