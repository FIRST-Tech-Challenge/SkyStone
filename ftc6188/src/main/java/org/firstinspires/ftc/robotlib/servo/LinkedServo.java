package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;
/*
This class defines two servos who are intended to operate as one
it really just simplifies the programming later since we then only have to set one position instead of two
 */
public class LinkedServo extends ModifiedServo
{
    // Stores the second servo, the first is already available in modified servo
    private Servo servoTwo;

    LinkedServo(Servo servoOne, Servo servoTwo)
    {
        super(servoOne);
        this.servoTwo = servoTwo;
    }

    @Override
    public void setPosition(double position)
    {
        this.servo.setPosition(position);
        servoTwo.setPosition(position);
    }

    public double getPosition2() { return servoTwo.getPosition(); }

    public Direction getDirection2() { return servoTwo.getDirection(); }

    public Servo getServoOne() { return this.servo; }

    public Servo getServoTwo() { return servoTwo; }
}
