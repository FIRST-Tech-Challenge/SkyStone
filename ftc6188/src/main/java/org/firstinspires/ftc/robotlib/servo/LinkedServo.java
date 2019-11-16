package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;
/*
This class defines two servos who are intended to operate as one
it really just simplifies the programming later since we then only have to set one position instead of two
 */
public class LinkedServo
{
    // Stores the reference to the servos defined in the hardware map
    private Servo servoOne;
    private Servo servoTwo;

    // True if the servos are facing at each other or their rotation is opposite
    private boolean oppositeFace;

    // The positions for servo one and two respectively
    private double position;
    private double position2;

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
        position2 = position;

        // inverts the position2 if the servos rotation is opposite (facing at each other)
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

    public double getPosition() { return position; }

    public double getPosition2() { return position2; }

    public Servo getServoOne() { return servoOne; }

    public Servo getServoTwo() { return servoTwo; }
}
