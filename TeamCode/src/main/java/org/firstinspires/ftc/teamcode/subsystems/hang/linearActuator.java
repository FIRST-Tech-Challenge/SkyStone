package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;

public class linearActuator implements Hang{

    private final float HANG_POWER = (float) 1.0;
    private final int HANG_DISTANCE = 14000;

    private DcMotor hanger;

    public linearActuator(DcMotor linearActuator)
    {
        hanger = linearActuator;
    }

    public void up() throws InterruptedException
    {
        oneMotorEncoder(hanger, HANG_POWER, HANG_DISTANCE);
    }

    public void down() throws InterruptedException
    {
        oneMotorEncoder(hanger, -HANG_POWER, -HANG_DISTANCE);


    }
}
