package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;





public class Intake extends Subsystem {
    Servo wrist;
    Servo main;


    public Intake(HardwareMap hardwareMap) {
        wrist = hardwareMap.servo.get("wrist");
        main = hardwareMap.servo.get("main");
        initServos(new Servo[]{wrist, main});
        initServo();

    }


    public void initServo() {

    }

}