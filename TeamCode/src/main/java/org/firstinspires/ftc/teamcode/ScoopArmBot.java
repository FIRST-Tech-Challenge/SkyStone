package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoopArmBot extends PinchArmBot {

    public Servo servoScoop = null;
    double  position = 0.1; // Start at halfway position

    public ScoopArmBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        servoScoop = hwMap.servo.get("servoScoop");

        servoScoop.setPosition(position);
        opMode.telemetry.log().add(String.format("scoopArm init pos %.2f", servoScoop.getPosition()));
    }

    public void scoopStone(){
        opMode.telemetry.log().add(String.format("scoopStone start pos : %.2f", servoScoop.getPosition()));

        servoScoop.setPosition(0.4);
        opMode.sleep(2000);
        opMode.telemetry.log().add(String.format("scoopStone %.2f", servoScoop.getPosition()));

        servoScoop.setPosition(0.7);
        opMode.sleep(2000);
        opMode.telemetry.log().add(String.format("scoopStone %.2f", servoScoop.getPosition()));

        servoScoop.setPosition(1.0);
        opMode.sleep(2000);
        opMode.telemetry.log().add(String.format("scoopStone %.2f", servoScoop.getPosition()));
    }

    public void scoopSetPosition(boolean buttonB, boolean buttonY, boolean buttonX) {
        if (buttonB) {
            servoScoop.setPosition(0.1);
        } else if (buttonY) {
            servoScoop.setPosition(0.4);
        } else if (buttonX) {
            servoScoop.setPosition(0.7);
        }
    }

    public void scoopFreeRun(boolean rightBumper, boolean leftBumper) {
        if (rightBumper) {
            servoScoop.setPosition(servoScoop.getPosition() + 0.005);
        } else if (leftBumper) {
            servoScoop.setPosition(servoScoop.getPosition() - 0.005);
        }

    }
}
