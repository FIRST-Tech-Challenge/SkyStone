package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoupArmBot extends FourWheelsDriveBot {

    public Servo servoScoup = null;
    double  position = 0.1; // Start at halfway position

    public ScoupArmBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        servoScoup = hwMap.servo.get("servoScoup");

        servoScoup.setPosition(position);
        opMode.telemetry.log().add(String.format("scoupArm init pos %.2f", servoScoup.getPosition()));
    }

    public void scoupStone(){
        opMode.telemetry.log().add(String.format("scoupStone start pos : %.2f", servoScoup.getPosition()));

        servoScoup.setPosition(0.4);
        opMode.sleep(2000);
        opMode.telemetry.log().add(String.format("scoupStone %.2f", servoScoup.getPosition()));

        servoScoup.setPosition(0.7);
        opMode.sleep(2000);
        opMode.telemetry.log().add(String.format("scoupStone %.2f", servoScoup.getPosition()));

        servoScoup.setPosition(1.0);
        opMode.sleep(2000);
        opMode.telemetry.log().add(String.format("scoupStone %.2f", servoScoup.getPosition()));
    }

}
