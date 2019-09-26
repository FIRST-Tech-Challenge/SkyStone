package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import android.os.Handler;
import android.os.SystemClock;
import android.util.Log;
import android.view.ViewParent;

public class JoystickTest extends OpMode {
    DcMotor frontleft, frontright, backleft, backright, collectorOne, collectorTwo, liftleft, liftright; //TODO convert to motormap @jake
    Servo flag, clamp, dump;
    public float x, y, z, w, pwr;
    public static double deadzone = 0.2;
    //remember, has to be float

    //servo minmax
    final static double CLAMP_MIN  = 172.0/255.0;
    final static double CLAMP_MID = 220.0/255.0;
    final static double CLAMP_MAX  = 232.0/255.0;
    final static double DUMP_MIN  = 80.0/255.0;
    final static double DUMP_MAX  = 180.0/255.0;

    @Override
    public void init() {
        frontleft = hardwareMap.dcMotor.get("front_left");
        frontright = hardwareMap.dcMotor.get("front_right");
        backleft = hardwareMap.dcMotor.get("back_left");
        backright = hardwareMap.dcMotor.get("back_right");
        liftleft = hardwareMap.dcMotor.get("lift_left");
        liftright = hardwareMap.dcMotor.get("lift_right");
        collectorOne = hardwareMap.dcMotor.get("collector_one");
        collectorTwo = hardwareMap.dcMotor.get("collector_two");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);
        //reverses right side, so collector is front

        flag = hardwareMap.servo.get("flag");
        dump = hardwareMap.servo.get("dump");
        clamp = hardwareMap.servo.get("clamp");
        dump.setPosition(DUMP_MIN);
        clamp.setPosition(CLAMP_MIN);
        //gets servos and sets to minimum values
    }

    @Override
    public void loop() {
        getJoyVals();
        //updates joyvalues with deadzones, xyzw

        pwr = y; //this can be tweaked for exponential power increase

        frontright.setPower(Range.clip(pwr - x+z, -1, 1));
        backleft.setPower(Range.clip(pwr - x-z, -1, 1));
        frontleft.setPower(Range.clip(pwr + x-z, -1, 1));
        backright.setPower(Range.clip(pwr + x+z, -1, 1));

        liftleft.setPower(w);
        liftright.setPower(w);
        //allows control of linear slides

        collectorOne.setPower((gamepad1.left_trigger>0.5)? -1:(gamepad1.left_bumper)? 1:0);
        collectorTwo.setPower((gamepad1.left_trigger>0.5)? -1:(gamepad1.left_bumper)? 1:0);
        //allows control of collector

        if(gamepad1.right_bumper) dump.setPosition(DUMP_MIN);
        if(gamepad1.right_trigger>0.5) dump.setPosition(DUMP_MAX);
        //allows control of dump servos

        if(gamepad1.a) clamp.setPosition(CLAMP_MIN);
        if(gamepad1.y) clamp.setPosition(CLAMP_MID);
        if(gamepad1.x) clamp.setPosition(CLAMP_MAX);
        //allows control of clamp servos
    }

    public void getJoyVals()
    {
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        z = gamepad1.right_stick_x;
        w = gamepad1.right_stick_y;
        //updates joystick values

        if(Math.abs(x)<deadzone) x = 0;
        if(Math.abs(y)<deadzone) y = 0;
        if(Math.abs(z)<deadzone) z = 0;
        if(Math.abs(w)<0.9) w = 0;
        //checks deadzones
    }


    @Override
    public void stop() {
        //nothing here? probably gotta call garbage collection at some point
    }
}