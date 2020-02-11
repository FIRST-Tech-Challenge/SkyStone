package org.firstinspires.ftc.teamcode.robots;

import android.graphics.Paint;
import android.hardware.Sensor;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.DigitalColourSensor;
import org.firstinspires.ftc.teamcode.newhardware.FXTSensors.FXTAnalogUltrasonicSensor;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.roboticslibrary.TaskHandler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Joules;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;​
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import org.firstinspires.ftc.robotcore.external.Func;​
import java.util.Locale;



public class Joules  {
    public Motor LinearSlides;
    public Motor FrontRight;
    public Motor FrontLeft;
    public Motor BackRight;
    public Motor BackLeft;
    private String VEER_CHECK_TASK_KEY = "Joules.VEERCHECK";

    public Boolean ScissorUp = Boolean.FALSE;
    public Boolean ScissorDown = Boolean.FALSE;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    //arm servoes
    private FXTServo Foundation;
    private FXTServo TapeMeasure;
    public FXTServo ScissorLift;

    private FXTServo StoneMover;
    //arm motoraaa  a

    //Capstone
    private FXTServo Capstone;
    private FXTCRServo Daffy;


    public static int STONESTATE;
    private float GEAR_RATIO = 1/2;

    public void init(){
        Foundation.setPosition(0.2);
        Capstone.setPosition(0.05);
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

   }

    public Joules(){
        // don't know if we need super();
        //Motors!!
        FrontRight = new Motor("frontR");
        FrontLeft = new Motor("frontL");
        BackRight = new Motor("backR");
        BackLeft = new Motor("backL");
        LinearSlides = new Motor("Linear Slides");

        Foundation = new FXTServo("Foundation mover");

        Daffy = new FXTCRServo("Daffy");

        TapeMeasure = new FXTServo("Tape Measure");

        StoneMover = new FXTServo("Stone mover");

        ScissorLift = new FXTServo("ScissorLift");

        Capstone = new FXTServo("Capstone");

        FrontRight.setMinimumSpeed(0.1);
        FrontLeft.setMinimumSpeed(0.1);
        BackRight.setMinimumSpeed(0.1);
        BackLeft.setMinimumSpeed(0.1);

    }

    public void SlidesUp() {
        LinearSlides.setPower(1);
    }

    public void SlidesDown() {
        LinearSlides.setPower(-1);
    }


    public void SlidesStop() {
        LinearSlides.setPower(0);
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
        Capstone.setPosition(1);
    }
    public void CapUp(){
        Capstone.setPosition(0.05);
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

    public void ScissorLiftDown(){ScissorLift.setPosition(0.2);}
    public void ScissorLiftUp(){ScissorLift.setPosition(0.8);}
    public void ScissorLiftEst(){ScissorLift.setPosition(0.7);}//subject to chnage


    public double ScissorValues() {return ScissorLift.getPosition();}

    public void ScissorLiftOut(){ScissorLift.setPosition(ScissorLift.getPosition() + 0.005);}


    public void ScissorLiftIn(){ScissorLift.setPosition(ScissorLift.getPosition() - 0.005);}




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
        Daffy.setPower(0.6);
    }
    public void DaffyGrab(){
        Daffy.setPower(-0.6);
    }
    public void DaffyStop() {Daffy.setPower(0);}



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
