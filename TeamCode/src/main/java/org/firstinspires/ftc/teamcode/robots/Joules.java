package org.firstinspires.ftc.teamcode.robots;

import android.graphics.Paint;
import android.hardware.Sensor;

import com.qualcomm.ftccommon.configuration.ConfigureFromTemplateActivity;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.DigitalColourSensor;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.FXTAnalogUltrasonicSensor;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.roboticslibrary.TaskHandler;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;


public class Joules  {
    public Motor FrontRight;
    public Motor FrontLeft;
    public Motor BackRight;
    public Motor BackLeft;
    private String VEER_CHECK_TASK_KEY = "Joules.VEERCHECK";

    //arm servoes
    private FXTServo Foundation;
    private FXTServo TapeMeasure;

    private FXTServo StoneMover;
    //arm motoraaa  a

    //Capstone
    private FXTServo Capstone;
    private FXTServo Daffy;
    private FXTCRServo ChainArm;

    public static int STONESTATE;
    private float GEAR_RATIO = 1/2;

    public void init(){
        Foundation.setPosition(0.2);
        Capstone.setPosition(0.05);
   }

    public Joules(){
        // don't know if we need super();
        //Motors!!
        FrontRight = new Motor("frontR");
        FrontLeft = new Motor("frontL");
        BackRight = new Motor("backR");
        BackLeft = new Motor("backL");

        Foundation = new FXTServo("Foundation mover");

        Daffy = new FXTServo("Box grabber");

        TapeMeasure = new FXTServo("Tape Measure");

        Capstone = new FXTServo("Capstone");

        StoneMover = new FXTServo("Stone mover");

        ChainArm =  new FXTCRServo("Scoring arm");

        FrontRight.setMinimumSpeed(0.1);
        FrontLeft.setMinimumSpeed(0.1);
        BackRight.setMinimumSpeed(0.1);
        BackLeft.setMinimumSpeed(0.1);
    }


    //Robot driving
    public void DriveForward(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(speed);
    }
    public void DriveBackward(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(speed);
        BackRight.setPower(-speed);

    }
    public void StrafeLeft(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(-speed);
    }
    public void StrafeRight(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(speed);
        BackRight.setPower(speed);
    }
    public void TurnLeft(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(-speed);

    }
    public void TurnRight(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(speed);
        BackRight.setPower(speed);
    }
    public void Stop(){
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void LeftPivot(double speed){
        FrontLeft.setPower(0);
        FrontRight.setPower(-speed);
        BackLeft.setPower(0);
        BackRight.setPower(-speed);
    }
    public void RightPivot(double speed){
        FrontLeft.setPower(speed);
        FrontRight.setPower(0);
        BackLeft.setPower(speed);
        BackRight.setPower(0);
    }

    public long getSeconds(double Voltage, int Seconds){
        return (long)((Seconds-((50*Voltage)-600)));
    }

    //Capstone
    public void CapDown(){
        Capstone.setPosition(0.05);
    }
    public void CapUp(){
        Capstone.setPosition(1);
    }


    //foundation
    public void FoundationDrop(){
        Foundation.setPosition(0.25);
    }
    public void FoundationGrab(){
        Foundation.setPosition(0.47);
    }

    public void TapeMeasureSpring() {TapeMeasure.setPosition(0.8);}
    public void TapeMeasurePush() {TapeMeasure.setPosition(0.2);}

    public void StoneDown(){
        StoneMover.setPosition(0.1);

    }
    public void StoneUp(){
        StoneMover.setPosition(0.7);

    }
   // public void StoneStop(){
       // StoneMover.Po(0);
  //  }

    public void DaffyUp(){
        Daffy.setPosition(0.4);
    }
    public void DaffyGrab(){
        Daffy.setPosition(1);
    }

    public void SlidesUp(){
        ChainArm.setPower(-0.7);
    }
    public void SlidesDown(){
        ChainArm.setPower(0.7);
    }
    public void SlidesStop(){
        ChainArm.setPower(0);
    }

    public void DriveForwardEnc(double speed, int distance){
//        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
//        FrontLeft.resetEncoder();
//        FrontRight.resetEncoder();
//        BackLeft.resetEncoder();
//        BackRight.resetEncoder();

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        RC.t.addData("abs1", FrontLeft.getAbsolutePosition());
//        RC.t.addData("current1", FrontLeft.getBaseCurrentPosition());

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        RC.t.addData("abs2", FrontLeft.getAbsolutePosition());
//        RC.t.addData("curren2t", FrontLeft.getBaseCurrentPosition());

        FrontLeft.setTarget(-distance);
        FrontRight.setAbsoluteTarget(distance);
        BackLeft.setAbsoluteTarget(-distance);
        BackRight.setAbsoluteTarget(distance);

        DriveForward(speed);

//        RC.t.addData("abs", FrontLeft.getAbsolutePosition());
//        RC.t.addData("current", FrontLeft.getBaseCurrentPosition());

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void DriveBackwardEnc(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(speed);
        BackRight.setPower(-speed);

    }
    public void StrafeLeftEnc(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(-speed);
    }
    public void StrafeRightEnc(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(speed);
        BackRight.setPower(speed);
    }
    public void TurnLeftEnc(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(-speed);

    }
    public void TurnRightEnc(double speed){
        TaskHandler.pauseTask(VEER_CHECK_TASK_KEY);
        FrontLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(speed);
        BackRight.setPower(speed);
    }



}
