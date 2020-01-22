package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoDropBot extends ButtArmBot{
    public Servo servoDrop = null;
    double position = 0.0;

    public servoDropBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        servoDrop = hwMap.servo.get("servoDrop");

        servoDrop.setPosition(position);
    }
    public void toggleServoDrop(boolean Abutton, boolean Bbutton){
        if(Abutton){
            servoDrop.setPosition(position+0.3);
        }
        if (Bbutton) {
            servoDrop.setPosition(position);
        }
    }


}
