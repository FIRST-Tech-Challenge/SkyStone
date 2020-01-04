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

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.DigitalColourSensor;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.FXTAnalogUltrasonicSensor;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.roboticslibrary.TaskHandler;
import com.qualcomm.robotcore.hardware.ColorSensor;


public class Joules  {
    private Motor FrontRight;
    private Motor FrontLeft;
    private Motor BackRight;
    private Motor BackLeft;
    private String VEER_CHECK_TASK_KEY = "Joules.VEERCHECK";

    //arm servoes
    private FXTServo Foundation1;
    private FXTServo Foundation2;

    private Motor StoneMover;
    //arm motoraaa  a

    //Capstone
    private FXTServo Capstone;

    public static int STONESTATE;
    private float GEAR_RATIO = 1/2;


    public Joules(){
        // don't know if we need super();
        //Motors!!
        FrontRight = new Motor("FrontRight");
        FrontLeft = new Motor("FrontLeft");
        BackRight = new Motor("BackRight");
        BackLeft = new Motor("BackLeft");


        Foundation1 = new FXTServo("Foundation1");
        Foundation2 = new FXTServo("Foundation2");

        Capstone = new FXTServo("Capstone");

        StoneMover = new Motor("StoneMover");


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

    //Capstone
    public void CapDown(){
        Capstone.setPosition(0.1);
    }
    public void CapUp(){
        Capstone.setPosition(0.9);
    }


    //foundation
    public void FoundationDrop(){
        Foundation1.setPosition(0.2);
        Foundation2.setPosition(0.2);
    }
    public void FoundationGrab(){
        Foundation1.setPosition(0.44);
        Foundation2.setPosition(0.44);
    }

    public void StoneDown(){
        StoneMover.setPower(-0.2);

    }
    public void StoneUp(){
        StoneMover.setPower(0.3);

    }
    public void StoneStop(){
        StoneMover.setPower(0);
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

        RC.t.addData("abs1", FrontLeft.getAbsolutePosition());
        RC.t.addData("current1", FrontLeft.getBaseCurrentPosition());

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RC.t.addData("abs2", FrontLeft.getAbsolutePosition());
        RC.t.addData("curren2t", FrontLeft.getBaseCurrentPosition());

        FrontLeft.setTarget(-distance);
        FrontRight.setAbsoluteTarget(distance);
        BackLeft.setAbsoluteTarget(-distance);
        BackRight.setAbsoluteTarget(distance);

        DriveForward(speed);

        RC.t.addData("abs", FrontLeft.getAbsolutePosition());
        RC.t.addData("current", FrontLeft.getBaseCurrentPosition());

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
