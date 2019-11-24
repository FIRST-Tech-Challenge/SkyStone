package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BBLift {

    private DcMotor _leftLift;
    private DcMotor _rightLift;

    private Servo _gripServo;
    private Servo _gripServo2;

    private int leftLiftStartPos;
    private int rightLiftStartPos;

    private Telemetry _telementary;

    private LinearOpMode _opMode;

    public void init(HardwareMap hwmap, Telemetry tele,LinearOpMode opMode ){

        _telementary = tele;
        _opMode = opMode;

        _leftLift = hwmap.get(DcMotor.class, "left_lift");
        _leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _leftLift.setDirection(DcMotor.Direction.REVERSE);
        _leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _rightLift = hwmap.get(DcMotor.class, "left_lift");
        _rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _rightLift.setDirection(DcMotor.Direction.FORWARD);
        _rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _gripServo = hwmap.get(Servo.class, "grip_servo");
        _gripServo2 = hwmap.get(Servo.class, "grip_servo2");

        _leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftStartPos = _leftLift.getCurrentPosition();
        rightLiftStartPos = _rightLift.getCurrentPosition();


    }

    public void LoadBrick(){

        Release();

        _opMode.sleep(250);

        //move down to starting pos on the encoders.

        _rightLift.setDirection(DcMotor.Direction.REVERSE);
        _leftLift.setDirection(DcMotor.Direction.FORWARD);


        _leftLift.setTargetPosition(leftLiftStartPos - 30);
        _rightLift.setTargetPosition(rightLiftStartPos + 30);

        _rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _leftLift.setPower(0.5);
        _rightLift.setPower(0.5);


        while (_opMode.opModeIsActive() && _leftLift.isBusy() && _rightLift.isBusy())
        {
            _telementary.addData("L-encoder-fwd", _leftLift.getCurrentPosition() + "  busy=" + _leftLift.isBusy());
            _telementary.addData("R-encoder-fwd", _rightLift.getCurrentPosition() + "  busy=" + _rightLift.isBusy());

            _telementary.update();
            _opMode.idle();
        }

        _leftLift.setPower(0);
        _rightLift.setPower(0);


        //then grab the brick

        Grip();
        _opMode.sleep(500);

        //wait back in the home position
        GoToHome();

    }

    public void GoToHome(){

        _rightLift.setDirection(DcMotor.Direction.REVERSE);
        _leftLift.setDirection(DcMotor.Direction.FORWARD);


        _leftLift.setTargetPosition(leftLiftStartPos + 250);
        _rightLift.setTargetPosition(rightLiftStartPos - 250);

        _rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        _leftLift.setPower(0.5);
        _rightLift.setPower(0.5);


        while (_opMode.opModeIsActive() && _leftLift.isBusy() && _rightLift.isBusy())
        {
            _telementary.addData("L-encoder-fwd", _leftLift.getCurrentPosition() + "  busy=" + _leftLift.isBusy());
            _telementary.addData("R-encoder-fwd", _rightLift.getCurrentPosition() + "  busy=" + _rightLift.isBusy());

            _telementary.update();
            _opMode.idle();
        }

       // _leftLift.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
       // _rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _leftLift.setPower(0);
        _rightLift.setPower(0);


    }

    public void Home(){

        _rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _rightLift.setDirection(DcMotor.Direction.REVERSE);
        _leftLift.setDirection(DcMotor.Direction.FORWARD);
        _leftLift.setPower(0.5);
        _leftLift.setPower(0.5);

    }

    public void DropOff()
    {
        _rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _rightLift.setDirection(DcMotor.Direction.FORWARD);
        _leftLift.setDirection(DcMotor.Direction.REVERSE);
        _leftLift.setPower(0.5);
        _leftLift.setPower(0.5);
    }

    public void Off(){
        _leftLift.setPower(0);
        _rightLift.setPower(0);
    }

    public void Grip(){
        _gripServo.setPosition(1);
        _gripServo2.setPosition(0);
    }

    public void Release(){
        _gripServo.setPosition(0);
        _gripServo2.setPosition(1);
    }
}
