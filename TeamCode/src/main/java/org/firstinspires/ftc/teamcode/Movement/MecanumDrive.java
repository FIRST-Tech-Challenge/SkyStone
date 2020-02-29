package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

public class MecanumDrive extends Drivebase {

    private LinearOpMode opMode;
    private RobotHardware hardware;

    public MecanumDrive(LinearOpMode opMode, RobotHardware hardware){
        this.opMode = opMode;
        this.hardware = hardware;
    }

    @Override
    public void initialize(){
        lf = 0;
        rf = 0;
        lb = 0;
        rb = 0;

        reverseMotors();
        setRunMode("withEncoder");
        setPowerBehavior("brake");
    }

    @Override
    public void update(){
        if(opMode.opModeIsActive()){
            hardware.rightFront.setPower(rf);
            hardware.leftFront.setPower(lf);
            hardware.leftBack.setPower(lb);
            hardware.rightBack.setPower(rb);
        }
    }

    public void setRelativeVelocity(double velX, double velY, double velHeading){
        /* Wheel movement to move horizontally
          A          |
          |          V
               -->
          |          A
          V          |
        */
        lf = velY + velX*1.3 - velHeading;
        rf = velY - velX*1.3 + velHeading;
        lb = velY - velX*1.3 - velHeading;
        rb = velY + velX*1.3 + velHeading;

    }

    @Override
    public void freeze(){

        lf = 0;
        rf = 0;
        lb = 0;
        rb = 0;
        update();

    }

    private void setPowerBehavior(String behavior){

        if(behavior.equals("brake")){
            hardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else if(behavior.equals("coast")){
            hardware.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hardware.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hardware.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    private void setRunMode(String runMode){

        if(runMode.equals("withEncoder")){
            hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(runMode.equals("withoutEncoder")){
            hardware.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void resetDriveEncoders(){

        hardware.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void reverseMotors(){

        // Reverse the necessary motors so that when positive power is set to all four, the robot moves forward
        hardware.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        hardware.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

}
