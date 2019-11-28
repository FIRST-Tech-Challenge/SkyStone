package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;

public class BoundServo extends ModifiedServo
{
    // A reference to what the servo really is
    private int openPos;
    private int closePos;

    public BoundServo (Servo servo, int openPos, int closePos)
    {
        super(servo);
        this.openPos = openPos;
        this.closePos = closePos;
    }

    public BoundServo(Servo servo)
    {
        this(servo, 1, 0);
    }

    public void openPosition() { setPosition(openPos); }

    public void closePosition() { setPosition(closePos); }
}
