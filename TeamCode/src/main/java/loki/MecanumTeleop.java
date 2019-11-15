package loki;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumTeleop extends OpMode {
    public DcMotor lf, rf, lb, rb, ls; //Define Motors In Code
    public Gamepad g1, g2;
    public Servo clawL, clawR, hook;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime hookTime = new ElapsedTime();
    ColorSensor color_sensor;
    float hsvValues[] = {0F, 0F, 0F};
    boolean up = true;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        //Motor Define In Phone
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        ls = hardwareMap.dcMotor.get("ls");
        //Servo Define
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");

        hook = hardwareMap.servo.get("hook");

        //color_sensor = hardwareMap.colorSensor.get("color");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Runs based on speed instead of voltage; makes run more consistently
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ls.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop(){
        final double x = gamepad1.left_stick_x;
        final double y = gamepad1.left_stick_y;
        double direction = Math.sqrt(x * x + y * y);
        //returns hypotonuse (C value in triangle)

        double robotAngle = Math.atan2(y, x);
        //return angle x (next to center of circle)

        double rightX = gamepad1.right_stick_x; //reverses rotation
        //rotiation

        final double lfPow = direction * Math.cos(robotAngle - Math.PI / 4.0) - rightX;
        final double rfPow = direction * Math.sin(robotAngle - Math.PI / 4.0) + rightX;
        final double lbPow = direction * Math.sin(robotAngle - Math.PI / 4.0) - rightX;
        final double rbPow = direction * Math.cos(robotAngle - Math.PI / 4.0) + rightX;
        //determines wheel power


        rf.setPower(rfPow);
        rb.setPower(rbPow);
        lf.setPower(lfPow);
        lb.setPower(lbPow);

        telemetry.addData("Robot Angle",robotAngle*(180/Math.PI));
    }
}

