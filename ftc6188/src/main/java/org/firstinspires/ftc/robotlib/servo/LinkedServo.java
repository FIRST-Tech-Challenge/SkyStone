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

    private void updateServoPosition()
    {
        servoOne.setPosition(position);
        servoTwo.setPosition(position);
    }
}
