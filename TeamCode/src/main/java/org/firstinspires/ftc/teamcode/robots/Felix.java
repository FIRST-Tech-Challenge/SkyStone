package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.Motor;

/**
 * Created by Aila on 2017-12-06.
 */

public class Felix extends Robot{

    public DcMotor wheelL = null;
    public DcMotor wheelR = null;

    public DcMotor glifter = null;

    public Servo jewelL = null;
    public Servo jewelR = null;

    /*
    public CRServo handL = null;
    public CRServo handR = null;
    */

    public static final double WHEEL_SIZE = 4.0;

    public static double LEFT_JEWEL_UP = 1;
    public static double LEFT_JEWEL_DOWN = 0.2;
    public static double RIGHT_JEWEL_UP = 0.5;
    public static double RIGHT_JEWEL_DOWN = 0.0;

    public Felix() {
        init(RC.h);
    }

    public void init (HardwareMap ahwMap) {
        HardwareMap hwmap = ahwMap;

        reverseDriveSystem();

        wheelL = hwmap.get(DcMotor.class, "wheelL");
        wheelL.setPower(0);

        wheelR = hwmap.get(DcMotor.class, "wheelR");
        wheelR.setPower(0);

        glifter = hwmap.get(DcMotor.class, "glifter");
        glifter.setPower(0);

        jewelL = hwmap.get(Servo.class, "jewelL");
        jewelR = hwmap.get(Servo.class, "jewelR");

        jewelL.setPosition(1);
        jewelR.setPosition(1);

        /*
        handL = hwmap.get(CRServo.class, "handL");
        handR = hwmap.get(CRServo.class, "handR");
        */

        //releaseGlyph();

    }


    public void holdGlyph () {
        wheelL.setPower(-0.9);
        wheelR.setPower(0.9);
    }


    public void releaseGlyph () {
        /*
        handL.setPower(0.7);
        handR.setPower(-0.7);
        */
    }

    public void stop () {
        super.stop();

        wheelL.setPower(0);
        wheelR.setPower(0);

    }

    public void leftJewel (boolean up) {
        if (!up) {
            jewelL.setPosition(0.2);
        }
        else {
            jewelL.setPosition(1);
        }
    }

    public static void wait(int time) {
        if(RC.o instanceof LinearOpMode){
            RC.l.sleep(time);
        }
    }//wait

    public void lift (int ticks, double speed) {
        int pos = glifter.getCurrentPosition();
        int target = pos + ticks;
        glifter.setTargetPosition(target);
        while (glifter.getCurrentPosition() < target){
            glifter.setPower(speed);
        }
        glifter.setPower(0);
    }

    public void drop (int ticks, double speed) {
        int pos = glifter.getCurrentPosition();
        int target = pos - ticks;
        glifter.setTargetPosition(target);
        while (glifter.getCurrentPosition() > target){
            glifter.setPower(speed);
        }
        glifter.setPower(0);
    }

    public void gliftAuto () {
        //Lifts glyph ~3"
        glifter.setPower(0.4);
        wait(200);
        glifter.setPower(0);
    }
}
