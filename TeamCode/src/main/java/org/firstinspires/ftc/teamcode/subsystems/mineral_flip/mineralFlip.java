package org.firstinspires.ftc.teamcode.subsystems.mineral_flip;

import com.qualcomm.robotcore.hardware.Servo;

public class mineralFlip implements Flip {

    Servo mineralFlip;

    final double UP_POSITION = 0.5;
    final double DOWN_POSITION = 0.75;
    final double FLIP_POSITION = 0.27;

    public mineralFlip(Servo mineralFlip)
    {
        this.mineralFlip = mineralFlip; }

    @Override
    public void up() throws InterruptedException
    {
        mineralFlip.setPosition(UP_POSITION);
    }

    @Override
    public void down()
    {
        mineralFlip.setPosition(DOWN_POSITION);
    }

    @Override
    public void flip()
    {
        mineralFlip.setPosition(FLIP_POSITION);
    }
}