package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;

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

    public void setPosition(double position)
    {
        this.position = position;
        updateServoPosition();
    }

    public double getPosition() { return position; }

    public String getActual()
    {
        return ("One: " + servoOne.getPosition() + " Two: " + servoTwo.getPosition());
    }

    private void updateServoPosition()
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
        servoOne.setPosition(position);
    }
}
