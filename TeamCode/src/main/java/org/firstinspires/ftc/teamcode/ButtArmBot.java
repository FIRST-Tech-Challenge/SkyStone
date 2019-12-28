package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ButtArmBot extends ScoopArmBot{
    public Servo buttArm = null;
    double  position = 0.1; // Start at halfway position

    public ButtArmBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        buttArm = hwMap.servo.get("buttArm");

        buttArm.setPosition(position);
    }

    public void toggleButtArm(boolean rightStickButton, boolean leftStickButton) {
        if (rightStickButton) {
            buttArm.setPosition(0.2);
        }
        if (leftStickButton) {
            buttArm.setPosition(0.1);
        }


    }
}
