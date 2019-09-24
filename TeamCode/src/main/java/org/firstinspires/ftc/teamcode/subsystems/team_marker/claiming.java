package org.firstinspires.ftc.teamcode.subsystems.team_marker;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Rohan on 12/28/2018.
 */
public class claiming implements TeamMarker {

    Servo dropper;

    final double DROP_POSITION = 1.0;
    final double HOLD_POSITION = 0.2;

    public claiming(Servo teamMarkerArm){
        this.dropper = teamMarkerArm;
    }

    @Override
    public void drop() {
        dropper.setPosition(DROP_POSITION);
    }

    @Override
    public void hold() {
        dropper.setPosition(HOLD_POSITION);
    }

    public void setArmPosition(double position){
        dropper.setPosition(position);
    }
}
