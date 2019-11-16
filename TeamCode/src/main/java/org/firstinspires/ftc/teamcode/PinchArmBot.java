package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PinchArmBot extends FourWheelsDriveBot {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   500;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    public Servo servoArm = null;
    public Servo servoPinch = null;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    public PinchArmBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servoArm = hwMap.servo.get("servoArm");
        servoPinch = hwMap.servo.get("servoPinch");

        servoArm.setPosition(1.0);
        servoPinch.setPosition(0.0);
    }

    public void pickupSkyStone(){
        opMode.sleep(3*1000);
        return;

        opMode.telemetry.log().add("pickupSkyStone()");
        int i = 1;
        while (this.opMode.opModeIsActive() && i < 4) {
            opMode.telemetry.log().add(String.format("pickupSkystone loop %d, %.2f, %.2f", i, servoArm.getPosition(), servoPinch.getPosition()));

            servoArm.setPosition(servoArm.getPosition() - 0.5);
            servoPinch.setPosition(servoPinch.getPosition() + 0.3);
            opMode.sleep(2000);
            opMode.telemetry.log().add(String.format("pickupSkystone end %d, %.2f, %.2f", i, servoArm.getPosition(), servoPinch.getPosition()));
            opMode.idle();
            i++;
        }

    }

    public void dropSkyStone(){
        opMode.sleep(2*1000);
        return;
    }

    public void resetArm(){
        opMode.sleep(1*1000);
        return;
    }

}
