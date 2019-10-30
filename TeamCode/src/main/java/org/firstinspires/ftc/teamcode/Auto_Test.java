package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sqrt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class Auto_Test extends MecanumAutoCentral {

    private final double POWER = 0.5;

    public void runOpMode(){
        clamp = hardwareMap.servo.get("clamp");

        clamp.setPosition(1);
        clamp.setPosition(0.4);
    }
}
