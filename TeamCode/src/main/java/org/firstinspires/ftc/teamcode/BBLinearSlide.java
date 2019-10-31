package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BBLinearSlide
{
    private DcMotor _armMotor;
    private DcMotor _spool;

    private Servo _rotater;
    private Servo _grabber;
    private CRServo _leveller;

    //private CRServo _spoolServo;



    private int spoolStartingPos = 0;

    private int SPOOL_EXTENSION = 2400;

    private Telemetry _tele;

    public void init(HardwareMap hwmap, Telemetry telemetry){

        _tele = telemetry;
        _armMotor = hwmap.get(DcMotor.class, "arm_motor");
        _armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _spool = hwmap.get(DcMotor.class, "spool");
        _spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _rotater = hwmap.get(Servo.class, "rotater");
        _grabber = hwmap.get(Servo.class, "grabber");
        _leveller = hwmap.get(CRServo.class, "leveller");

        //_spoolServo = hwmap.get(CRServo.class, "spoolservo");



    }

    public void MoveUp()
    {
        //TODO: Arm speed ?
        _armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _armMotor.setPower(0.75);
    }

    public void MoveDown()
    {
        //TODO: Arm speed ?
        _armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _armMotor.setPower(0.75);
    }

    public void MoveDown(double speed)
    {

        _armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _armMotor.setPower(speed);
    }

    public void SlideIn()
    {
        _tele.addLine("Slide In");

        _tele.update();
        //_spoolServo.setDirection(DcMotorSimple.Direction.FORWARD);
       // _spoolServo.setPower(1);
        _spool.setDirection(DcMotorSimple.Direction.REVERSE);
        _spool.setPower(1);
    }

    public void SlideOut()
    {
        _tele.addLine("Slide Out");

        _tele.update();
        //_spoolServo.setDirection(DcMotorSimple.Direction.REVERSE);
        //_spoolServo.setPower(1);
        _spool.setDirection(DcMotorSimple.Direction.FORWARD);
        _spool.setPower(1);
    }



    public void StopArm()
    {
        _armMotor.setPower(0);
    }

    public void SetArmAutoPosAndHold()
    {
        //move the arm to the correct position for the auto phase. hold it using the encoders

        _armMotor.setTargetPosition(_armMotor.getCurrentPosition() - 600);
        _armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _armMotor.setPower(0.4);
    }

    public void StopSlide()
    {
        //_spoolServo.setPower(0);
        _spool.setPower(0);
    }

    public void RotateReset()
    {
        _rotater.setPosition(0);
    }

    public void Rotate() {
        _rotater.setPosition(1);
    }

    public void Level(double speed) {
        _leveller.setDirection(DcMotorSimple.Direction.FORWARD);
        _leveller.setPower(speed);
    }

    public void ReLevel(double speed) {
        _tele.addLine("RELEVEL");
        _tele.update();
        _leveller.setDirection(DcMotorSimple.Direction.REVERSE);
        _leveller.setPower(speed);
    }

    public void LevelStop(){
        _leveller.setPower(0);
    }

    public void Grab() {
        _grabber.setPosition(0);
    }

    public void Release() {
        _grabber.setPosition(1);
    }


}
