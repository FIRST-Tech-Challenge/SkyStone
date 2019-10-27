package org.firstinspires.ftc.robotlib.servo;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoManager {
    private Servo[] servos;

    public ServoManager(Servo[] servos) {
        this.servos = servos;
    }

    public void setPosition(double position) {
        for (Servo servo : servos) {
            servo.setPosition(position);
        }
    }
}
