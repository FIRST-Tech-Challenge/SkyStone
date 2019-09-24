package org.firstinspires.ftc.teamcode.subsystems.dunk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;

import static org.firstinspires.ftc.teamcode.subsystems.DriveFunctions.oneMotorEncoder;


public class dunkMinerals implements Dunk
{
    private DcMotor lifter;
    private Servo dunker;

    //Dunk Servo Variables
    private final double DUNK_POSITION = 0.25;
    private final double DOWN_POSTITION = 0.75;
    private final double HOLD_POSITION = 0.665;

    final float LIFT_POWER = (float) 1.0;
    final int UP_DISTANCE = 4450;
    final int DOWN_DISTANCE = 4400;

    public dunkMinerals(DcMotor lifter, Servo dunker)
    {
        this.lifter = lifter;
        this.dunker = dunker;
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Dunk
    @Override
    public void dunk()
    {
        dunker.setPosition(DUNK_POSITION);
    }

    @Override
    public void dunkDown()
    {
        dunker.setPosition(DOWN_POSTITION);
    }

    @Override
    public void dunkHold()
    {
        dunker.setPosition(HOLD_POSITION);
    }

    @Override
    public void liftUp() throws InterruptedException
    {
        dunkHold();
        oneMotorEncoder(lifter, LIFT_POWER, UP_DISTANCE);
    }

    @Override
    public void liftDown() throws InterruptedException
    {
        oneMotorEncoder(lifter, -LIFT_POWER, -DOWN_DISTANCE);
    }
}




