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

/*


// tank drive
// note that if y equal -1 then joystick is pushed all of the way forward.
float left = -gamepad1.left_stick_y;
float right = -gamepad1.right_stick_y;


Java
// write the values to the motors
motorRight.setPower(right);
motorLeft.setPower(left);
Before this you have to make sure to “clip” the joystick values to they never go above 1 and below -1, because those are the only value range that the motors now take.  To do this:
// clip the right/left values so that the values never exceed +/- 1
right = Range.clip(right, -1, 1);
left = Range.clip(left, -1, 1);

*/

public class JoystickTest extends OpMode {
    DcMotor frontleft, frontright, backleft, backright;
    public float x, y, z, w, pwr;
    public static double deadzone = 0.2;
    //    //remember, has to be float

    //servo minmax
    @Override
    public void init() {
        frontleft = hardwareMap.dcMotor.get("front_left");
        frontright = hardwareMap.dcMotor.get("front_right");
        backleft = hardwareMap.dcMotor.get("back_left");
        backright = hardwareMap.dcMotor.get("back_right");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);

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