package org.firstinspires.ftc.teamcode;

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
    private Servo _leveller;

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
        _leveller = hwmap.get(Servo.class, "leveller");


        //TODO: We want the slide in and out to use the encoder so that we don't travel too far
        //record the value of the starting encoder positon
        //then we will have a function that returns the motor to that position on the slide in.

        spoolStartingPos = Math.abs(_spool.getCurrentPosition());
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

    public void SlideIn()
    {
        _tele.addLine("Slide In");
        _tele.addData("Pos:", Math.abs(_spool.getCurrentPosition()));
        _tele.addData("Top Target:", (spoolStartingPos));

        _spool.setDirection(DcMotorSimple.Direction.FORWARD);

        //use the stored starting encoder position, use this to slide the motor back to this point
        //This means that a single touch will return the motor to the start.
        if(Math.abs(_spool.getCurrentPosition()) >= spoolStartingPos ) {

            _spool.setPower(1);
        }else{
            _spool.setPower(0);
           _tele.addLine("IN-STOP");
        }
        _tele.update();

    }

    public void SlideOut()
    {
        _tele.addLine("Slide Out");
        _tele.addData("Pos:", Math.abs(_spool.getCurrentPosition()));
        _tele.addData("Top Target:", (spoolStartingPos));

        _spool.setDirection(DcMotorSimple.Direction.REVERSE);
        if(Math.abs(_spool.getCurrentPosition()) <= (spoolStartingPos + SPOOL_EXTENSION)) {

            _spool.setPower(1);

        }else{
            _tele.addLine("OUT-STOP");
            _spool.setPower(0);
        }
        _tele.update();
    }



    public void StopArm()
    {
        _armMotor.setPower(0);
    }

    public void StopSlide(){
        _spool.setPower(0);
    }

    public void RotateReset() { _rotater.setPosition(0); }

    public void Rotate() {
        _rotater.setPosition(1);
    }

    public void Grab() {
        _grabber.setPosition(1);
    }

    public void Release() {
        _grabber.setPosition(0);
    }

    public void GripperForward() {
        _leveller.setPosition(_leveller.getPosition() + 0.1);
    }

    public void GripperBackward() {
        _leveller.setPosition(_leveller.getPosition() - 0.1);
    }
}
