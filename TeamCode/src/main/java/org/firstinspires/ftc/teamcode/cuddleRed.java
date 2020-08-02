package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class cuddleRed extends LinearOpMode {

    public DcMotor topLeft = null;
    public DcMotor topRight = null;
    public DcMotor botLeft = null;
    public DcMotor botRight = null;

    public DcMotor intakeR = null;
    public DcMotor intakeL = null;

    public DcMotor liftMotor = null;

    public Servo pullL = null;
    public Servo pullR = null;

    public Servo claw;

    public Servo armL;
    public Servo armR;

    public Orientation angles;
    public BNO055IMU imu;

    public void forward(int time, double speed){
        topLeft.setPower(speed);
        topRight.setPower(speed);
        botLeft.setPower(speed);
        botRight.setPower(speed);

        sleep(time);

        topLeft.setPower(0);
        topRight.setPower(0);
        botLeft.setPower(0);
        botRight.setPower(0);
    }

    public void backward(int time, double speed){
        topLeft.setPower(-speed);
        topRight.setPower(-speed);
        botLeft.setPower(-speed);
        botRight.setPower(-speed);

        sleep(time);

        topLeft.setPower(0);
        topRight.setPower(0);
        botLeft.setPower(0);
        botRight.setPower(0);
    }

    public void turnRight(int time, double speed){
        topLeft.setPower(speed);
        topRight.setPower(-speed);
        botLeft.setPower(speed);
        botRight.setPower(-speed);

        sleep(time);

        topLeft.setPower(0);
        topRight.setPower(0);
        botLeft.setPower(0);
        botRight.setPower(0);
    }

    public void turnLeft(int time, double speed){


        topLeft.setPower(-speed);
        topRight.setPower(speed);
        botLeft.setPower(-speed);
        botRight.setPower(speed);

        sleep(time);

        topLeft.setPower(0);
        topRight.setPower(0);
        botLeft.setPower(0);
        botRight.setPower(0);
    }

    public void foundationServosGrab(int time, double speed){
        double currTime = getRuntime();
        double targetTime = time/1000.0;
        while(getRuntime() <= currTime + targetTime){
            topLeft.setPower(speed);
            topRight.setPower(speed);
            botLeft.setPower(speed);
            botRight.setPower(speed);

            if(getRuntime() >= currTime + targetTime/2.0){
                pullL.setPosition(.32);
                pullR.setPosition(.84);
            }
        }

        topLeft.setPower(0);
        topRight.setPower(0);
        botLeft.setPower(0);
        botRight.setPower(0);

    }

    public void foundationServosDown(){
        pullL.setPosition(.32);
        pullR.setPosition(.84);
    }

    public void foundationServosUp(){
        pullL.setPosition(0.75);
        pullR.setPosition(0.38);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        topLeft = hardwareMap.dcMotor.get("topLeft");
        topRight = hardwareMap.dcMotor.get("topRight");
        botLeft = hardwareMap.dcMotor.get("botLeft");
        botRight = hardwareMap.dcMotor.get("botRight");

        pullL = hardwareMap.servo.get("pullL");
        pullR = hardwareMap.servo.get("pullR");

        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        botLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        topRight.setDirection(DcMotorSimple.Direction.REVERSE);
        botRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        forward(750, 0.7);

    }
}
